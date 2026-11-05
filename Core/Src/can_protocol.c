/*
 * can_protocol.c
 *
 *      Author: CT
 */

#include "can_protocol.h"

#include "can.h"
#include "foc_config.h"
#include "foc_control.h"

#define CMD_ENTER_MOTOR   0xFCu
#define CMD_EXIT_MOTOR    0xFDu
#define CMD_SET_ZERO      0xFEu
#define CMD_CALIBRATE     0xFBu
#define CMD_CLEAR_FAULTS  0xFAu
#define CMD_SET_ID        0xF9u

#define START_RETRY_MS    100u

static uint8_t s_id;
static uint8_t s_started;
static uint32_t s_last_start_try;

/**
 * Packs a float into an unsigned field of `bits` bits: clamp to [x_min, x_max],
 * then map that range linearly onto 0 .. 2^bits - 1 (rounded). The resolution
 * is (x_max - x_min) / (2^bits - 1), which is why the ranges in foc_config.h
 * should fit the motor.
 */
static uint32_t float_to_uint(float x, float x_min, float x_max, uint32_t bits)
{
  if (x < x_min)
  {
    x = x_min;
  }
  else if (x > x_max)
  {
    x = x_max;
  }
  const float full = (float)((1u << bits) - 1u);
  return (uint32_t)((x - x_min) * full / (x_max - x_min) + 0.5f);
}

/** Inverse of float_to_uint(): 0 -> x_min, 2^bits - 1 -> x_max. */
static float uint_to_float(uint32_t x, float x_min, float x_max, uint32_t bits)
{
  const float full = (float)((1u << bits) - 1u);
  return (float)x * (x_max - x_min) / full + x_min;
}

/**
 * Programs hardware filter bank 0 so only standard data frames with id == s_id
 * reach RX FIFO 0. Everything else on the bus (other motors' traffic) is dropped
 * by the CAN peripheral without waking the CPU.
 *
 * 32-bit mask mode: the id sits in bits 31..21 of the filter register (<< 5 in
 * the high half-word). The mask compares all 11 id bits plus the IDE
 * (extended) and RTR (remote) bits, which must be 0.
 */
static void config_filter(void)
{
  /* Accept only standard data frames addressed to s_id. */
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0u;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = (uint32_t)s_id << 5;
  f.FilterIdLow = 0u;
  f.FilterMaskIdHigh = 0x7FFu << 5;
  f.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;   /* IDE and RTR must be 0 */
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  (void)HAL_CAN_ConfigFilter(&hcan, &f);
}

/**
 * Takes the CAN peripheral out of init mode onto the bus. The peripheral first
 * waits for 11 recessive bits, so with no bus or an unpowered transceiver
 * HAL_CAN_Start times out. That isn't fatal: the motor stays safe in IDLE and
 * CanProto_Poll() retries every START_RETRY_MS. A failed start leaves the HAL
 * handle in ERROR state, so the peripheral is re-initialised before retrying.
 */
static void try_start(void)
{
  s_last_start_try = HAL_GetTick();
  if (hcan.State != HAL_CAN_STATE_READY)
  {
    /* A failed start leaves the handle in ERROR: re-init before retrying. */
    (void)HAL_CAN_Init(&hcan);
    config_filter();
  }
  /* Fails if the bus never goes recessive (e.g. transceiver unpowered); retried from Poll. */
  s_started = (HAL_CAN_Start(&hcan) == HAL_OK);
}

/**
 * Called once from main() after FocControl_Init() (which provides the stored id).
 * Re-inits the peripheral with automatic bus-off recovery and automatic
 * retransmission (the .ioc leaves both off), sets the receive filter to this
 * node's id, and joins the bus.
 */
void CanProto_Init(uint8_t node_id)
{
  s_id = node_id;

  /* Not expressed in the .ioc: recover from bus-off on its own, and retransmit
   * frames that lose arbitration (several motors share the bus). */
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  (void)HAL_CAN_Init(&hcan);

  config_filter();
  try_start();
}

/**
 * Sends the 8-byte status frame to the master: position (16 bits), velocity
 * (12), torque (12), then state/calibrated and fault bits (layout in
 * can_protocol.h). Called once per received frame, so the master gets exactly
 * one reply per command. If all three TX mailboxes are busy (bus saturated),
 * the reply is skipped rather than blocking the main loop.
 */
static void send_reply(void)
{
  if (!s_started || (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0u))
  {
    return;
  }

  FocFeedback_t fb;
  FocControl_GetFeedback(&fb);

  const uint32_t p = float_to_uint(fb.p, FOC_P_MIN, FOC_P_MAX, 16u);
  const uint32_t v = float_to_uint(fb.v, FOC_V_MIN, FOC_V_MAX, 12u);
  const uint32_t t = float_to_uint(fb.t, FOC_T_MIN, FOC_T_MAX, 12u);

  uint8_t d[8];
  d[0] = s_id;
  d[1] = (uint8_t)(p >> 8);
  d[2] = (uint8_t)p;
  d[3] = (uint8_t)(v >> 4);
  d[4] = (uint8_t)(((v & 0xFu) << 4) | (t >> 8));
  d[5] = (uint8_t)t;
  d[6] = (uint8_t)((fb.state & 0x0Fu) | (fb.calibrated ? 0x10u : 0u));
  d[7] = fb.faults;

  CAN_TxHeaderTypeDef h = {0};
  h.StdId = FOC_CAN_MASTER_ID;
  h.IDE = CAN_ID_STD;
  h.RTR = CAN_RTR_DATA;
  h.DLC = 8u;
  h.TransmitGlobalTime = DISABLE;
  uint32_t mailbox;
  (void)HAL_CAN_AddTxMessage(&hcan, &h, d, &mailbox);
}

/**
 * Recognises the special command frames (FF FF FF FF FF FF xx cmd, listed in
 * can_protocol.h) and turns them into requests for the slow loop. Byte 6 must
 * be FF, except for set-id where it carries the new id.
 * Returns 1 if the frame was a special command, 0 if it should be decoded as a
 * normal impedance command.
 */
static uint8_t handle_special(const uint8_t *d)
{
  for (uint32_t i = 0u; i < 6u; i++)
  {
    if (d[i] != 0xFFu)
    {
      return 0u;
    }
  }

  switch (d[7])
  {
    case CMD_ENTER_MOTOR:  if (d[6] != 0xFFu) { return 0u; } FocControl_RequestMotorMode(); return 1u;
    case CMD_EXIT_MOTOR:   if (d[6] != 0xFFu) { return 0u; } FocControl_RequestIdle(); return 1u;
    case CMD_SET_ZERO:     if (d[6] != 0xFFu) { return 0u; } FocControl_RequestZero(); return 1u;
    case CMD_CALIBRATE:    if (d[6] != 0xFFu) { return 0u; } FocControl_RequestCalibration(); return 1u;
    case CMD_CLEAR_FAULTS: if (d[6] != 0xFFu) { return 0u; } FocControl_RequestClearFaults(); return 1u;
    case CMD_SET_ID:
      if (FocControl_SetCanId(d[6]))
      {
        s_id = d[6];
        config_filter();
      }
      return 1u;
    default:
      return 0u;
  }
}

/**
 * Decodes an impedance command. The five fields are packed big-endian into
 * 64 bits: p[16] | v[12] | kp[12] | kd[12] | t[12]. The 12-bit fields straddle
 * byte boundaries, hence the nibble shifts. Each field is scaled back to
 * physical units and the result goes to the slow loop. Also resets the command
 * timeout.
 */
static void handle_command(const uint8_t *d)
{
  const uint32_t p  = ((uint32_t)d[0] << 8) | d[1];
  const uint32_t v  = ((uint32_t)d[2] << 4) | (d[3] >> 4);
  const uint32_t kp = (((uint32_t)d[3] & 0xFu) << 8) | d[4];
  const uint32_t kd = ((uint32_t)d[5] << 4) | (d[6] >> 4);
  const uint32_t t  = (((uint32_t)d[6] & 0xFu) << 8) | d[7];

  FocCommand_t c;
  c.p_des = uint_to_float(p, FOC_P_MIN, FOC_P_MAX, 16u);
  c.v_des = uint_to_float(v, FOC_V_MIN, FOC_V_MAX, 12u);
  c.kp = uint_to_float(kp, FOC_KP_MIN, FOC_KP_MAX, 12u);
  c.kd = uint_to_float(kd, FOC_KD_MIN, FOC_KD_MAX, 12u);
  c.t_ff = uint_to_float(t, FOC_T_MIN, FOC_T_MAX, 12u);
  FocControl_SetCommand(&c);
}

/**
 * Called continuously from the main loop. Retries joining the bus if that
 * failed. Otherwise empties RX FIFO 0: each frame addressed to this node with
 * 8 data bytes is handled as a special command or an impedance command, then
 * answered with a status frame.
 * Polling (instead of a CAN interrupt) is enough because the main loop has
 * nothing else to do. Latency is a few microseconds, plus whatever the fast
 * and slow loops preempt.
 */
void CanProto_Poll(void)
{
  if (!s_started)
  {
    if ((HAL_GetTick() - s_last_start_try) >= START_RETRY_MS)
    {
      try_start();
    }
    return;
  }

  CAN_RxHeaderTypeDef h;
  uint8_t d[8];
  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0u)
  {
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &h, d) != HAL_OK)
    {
      break;
    }
    if ((h.IDE != CAN_ID_STD) || (h.RTR != CAN_RTR_DATA) || (h.StdId != s_id) || (h.DLC != 8u))
    {
      continue;
    }
    if (!handle_special(d))
    {
      handle_command(d);
    }
    send_reply();
  }
}

/*
 * can_protocol.h
 *
 * MIT mini-cheetah style CAN protocol (1 Mbit/s, standard 11-bit ids).
 *
 * Master -> motor (id = motor CAN id, 8 bytes):
 *   command: p[16] v[12] kp[12] kd[12] t_ff[12], big-endian bit packing,
 *            linear over the FOC_*_MIN..FOC_*_MAX ranges in foc_config.h
 *   special: FF FF FF FF FF FF FF FC  enter motor mode
 *            FF FF FF FF FF FF FF FD  exit motor mode (idle)
 *            FF FF FF FF FF FF FF FE  set current position as zero (idle only, saved)
 *            FF FF FF FF FF FF FF FB  run encoder calibration (idle only, motor must be free)
 *            FF FF FF FF FF FF FF FA  clear faults
 *            FF FF FF FF FF FF id F9  change CAN id (idle only, saved)
 *
 * Motor -> master (id = FOC_CAN_MASTER_ID, 8 bytes), sent after every frame received:
 *   [0] motor id  [1..2] p[16]  [3..4.hi] v[12]  [4.lo..5] t[12]
 *   [6] state (bits 0-3, FocState_t) | calibrated (bit 4)
 *   [7] fault bits (FOC_FAULT_*)
 * Bytes 0-5 match the original 6-byte reply.
 *
 *      Author: CT
 */

#ifndef INC_CAN_PROTOCOL_H_
#define INC_CAN_PROTOCOL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void CanProto_Init(uint8_t node_id);

/** Call from the main loop: handles received frames and sends replies. */
void CanProto_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_PROTOCOL_H_ */

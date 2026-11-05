Im adding notes here of the new things i need to learn or i have learned
PWM needs to be center aligned 

With edge-aligned PWM, the switching edges for each phase are all clustered at one end of the period (e.g. all at the start). The phase currents are chopping hard at that moment, and the ripple pattern is asymmetric across the period.

With center-aligned PWM, each phase’s ON time is symmetric around the middle of the period. There is usually a more predictable “quiet” region near the center (or at the counter top/bottom, depending on how you trigger) where the inductor current is easier to interpret. For FOC you want ADC samples that match the average current over the PWM window as closely as possible; symmetry makes that much easier.

to calculate the configurations of the PWM:
F-PWM = (F-CLK/[(ARR+1) * (PSC +1)])

Given that the desired FPWM is 20kHz and my FCLK is 48MHz, let the prescaler PSC = 0.
so the formula ends up being (ARR+1) = 48M/20k -> ARR+1 = 2400 -> ARR = 2399

Then by solving for the ARR value, we’ll get 2399. However, as mentioned earlier in the center-aligned Mode 3, we should use only half the calculated value for ARR or CCR registers to control the PWM signal’s frequency and duty cycle properly. Therefore, we’ll write 1200 in the ARR register to achieve a 20kHz PWM.

Setting the PWM on the same tim, adding dead time (10 is ok??  10/48MHz = 208, need to check the mosfets datashseet).
PWM timer has an interruption
PWM is center aligned

ADC gets trigger from timer after falling edge.
DMA half word



- foc_fast.c: 20 kHz current loop, fixed point (no FPU on the F042). Runs from the ADC DMA
  transfer-complete IRQ. ADC is triggered by TIM1 CH4 (OC4REF -> TRGO, PWM2) 80 ticks before the
  counter peak, so the scan is centred where all low-side FETs are on. Encoder read over SPI1 by
  direct register access inside the loop. Drops the phase with the highest duty and rebuilds it.
- foc_control.c: 2 kHz slow loop in PendSV: state machine, impedance control
  (tau = kp*(p_des-p) + kd*(v_des-v) + t_ff), calibration, Vbus/timeout checks, watchdog refresh.
- can_protocol.c: MIT mini-cheetah style protocol, 1 Mbit/s (see header for commands).
- nv_config.c: calibration + CAN id in the last flash page (linker FLASH = 31K).
- foc_config.h: every parameter. Check all the TODOs before powering a motor.
- Debug build uses -Og (at -O0 it does not fit in 32 KB). foc_fast.c is always -O2. 


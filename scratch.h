#if false

  
// TODO: Motor gain/trim (i.e., adjust for differences in left/right speed or stiction-break)
// TODO: Wheel encoders for feedback / auto-calibrate trim
// TODO: Floor-proximity and collision sensors
// TODO: Third-tier w/ additional sensors?
// TODO: Clean up wiring
// TODO: Easier onboard vs. usb power switching
// TODO: Settings persistence
// TODO: Allow a small anti-stiction reservoir (reset when the motor is turned off) to briefly (and mildly) boost the power when transitioning out of a full-stop
// TODO: Sometimes locks up after a fall (motor stuck running, no further variation or remote response)
//       (might have been related to a buffer overflow when writing to the remote display buffer)
// TODO: Separate params for coasting/braking modes
// TODO: Partially attenuate Angle PID Kp based on angle max IIR, allowing response
//       to become more subtle near balance, but immediately ramp up for correction.
//       Maybe other params are adaptive as well?
// TODO: Auto-tune pitch trim - observe average power when gThrottleBias == 0, slowly adjust gPitchTrim to bring the averaged pwmDutyAccumulator closer to 0
// TODO: Battery gauge: investigate BatterySense library, only sample when the motor is off, or at least not accelerating to a greater magnitude?
// TODO: Set a battery fault if the voltage falls below a critical level
// TODO: Use voltage sense to dynamically adjust the pwm range (at the least, the max), but use an IIR or when-motor-off sampling to avoid oscillation from motor draw

// Alt: pwmFreq: 400, Kp 6.9/0.3, p0.86/0/0.3, P_IIR 0.5, v8.0,0,0, V_IIR 0.5, SP_IIR 0.010, Throttle 100%, mSmooth 0.908,
//   Weak balance, somewhat jittery
// Alt: pwmFreq: 16k, Kp 3.7/0.3, p0.46/0/0.04, P_IIR: 0.5, v6/0/0, V_IIR 0.5, SP_IIR 0.010, Throttle 100%, mSmooth 0.908,
//   Seems a resilient (if rubberbandy) balance, driveable, somewhat resistant to sudden wheel blockage
//   pKd=0.07, P_IIR=1.0 seems to reduce jitter and be just enough responsive to moderate disturbances
//   Learning... overly aggressive Mahony Kp was at the heart of much of the jitter and instability.
//   D-smoothing (gDIIRWeight) further caused D to lag.  This might have produced a phase offset (lag)
//     that resulted in oscillation / orbiting in the state-space?
//   While keeping target speeds to < 5V average for the motors' benefit, we can use brief bursts up to full
//     supply voltage (~8.4V) for emergency correction. However, that should be limited to prevent motor damage.


  // rawSpeed is a number strictly within the range [0, 255]
  // Here, we use the roll as the speed, approaching full speed as we approach 90 degrees.
  // TODO: In practice, we would need to ramp much earlier to recover our balance
  // TODO: There's also a nonlinear (1/sin?) relation between angle and speed, as
  //       angles near to upright (0 degrees roll) need very little correction, but
  //       recovering from angles > 20 degrees or so needs significant acceleration.
  //       Note that it's actually accel needed to recover from a tip, not speed...
  //       Do we want to have an input mode that focuses on accel, or rely on the
  //       external source to compute accel?
  // TODO: Also note that to recover balance, we need to overshoot or otherwise
  //       integrate error rapidly, ensuring that we can obtain a neutral position
  //       before we max out speed and accel.
  // We get back to the root of PID... we really want to go from
  // (desired angle, current angle, error) to speed, with the error integration
  // providing the overshoot.
  // Ok, let's focus explicitly on a balance controller
  // Misc. note: to start moving from still, we actually need to reverse a bit.
  // this should fall out naturally, as 'move forward' actually translates to
  // 'maintain an N degree angle to accel, until we're at speed'.  Without an
  // encoder, we can target having the desired speed at angle 0, and we want
  // to keep the speed well under our max, so that we always have accel left
  // to balance. We might even switch to coasting mode for a boost if needed.

  int desiredAngle = 0.0f;
  float currentAngle = gOrientation.roll;
  // TODO: Subtract the angle error expected from the current acceleration
  float currentAngleAdjusted = currentAngle;
  // How far off are we?
  float errorAngle = currentAngleAdjusted - desiredAngle;
  float Kp = 1.0f;
  float Ki = 0.1f;
  float Kd = 0.0f;


  int rawSpeed = constrain(map((int)std::abs(gOrientation.roll), 0, 90, 0, 255), 0, 255);

  RelativeDirection direction = rawSpeed < 0.0f ? eBackward : eForward;

  // Map the speed to the PWM min/max range
  int constrainedSpeed = constrain(map((int)std::abs(rawSpeed), 0, 90, gPwmMinDuty, gPwmMaxDuty), gPwmMinDuty, gPwmMaxDuty);
  gPwmDutyCurrent = constrain(map((int)-gOrientation.roll, 0, 90, gPwmMinDuty, gPwmMaxDuty), gPwmMinDuty, gPwmMaxDuty);

  int m1 = 0, m2 = 0;
  if (gOrientation.roll < 0) {
    // Note: for fast-decay (coasting), one pin is pulled low, the other is high at the desired duty.
    // For slow-decay (braking), one pin is pulled high, the other is low at the desired duty.
    // Because analogWrite specified the high-side duty cycle, slow-decay mode means we invert both
    // of the duty cycles (i.e., 255 on on to hold it high, and 255-duty on the other, so that it is
    // LOW at the desired duty)
    direction = eForward;
    m2 = constrain(map((int)-gOrientation.roll, 0, 90, gPwmMinDuty, gPwmMaxDuty), gPwmMinDuty, gPwmMaxDuty);
  } else {
    direction = eBackward;
    m1 = constrain(map((int)gOrientation.roll, 0, 90, gPwmMinDuty, gPwmMaxDuty), gPwmMinDuty, gPwmMaxDuty);
  }

  // Adjust for braking (slow-decay) / coasting (fast-decay) drive modes.
  if (gHBridgeIdleMode == eBraking) {
    // To invert the H-Bridge input, invert the levels and also
    // swap the driven IO to maintain direction.
    int m1Temp = m1;
    m1 = 255 - m2;
    m2 = 255 - m1Temp;
  }

  static int startupPwmAttenuation = 0;
  m1 = constrain(m1, 0, startupPwmAttenuation);
  m2 = constrain(m2, 0, startupPwmAttenuation);
  if (startupPwmAttenuation < 255)
    ++startupPwmAttenuation;

  static float m1IIR = 0.0f;
  static float m2IIR = 0.0f;
  // We want to prevent a sudden jump from stopped to high duty
  // The IIR is supposed to smooth the ramp
  // But if we swap braking/coasting, or if we reverse direction,
  // the values both start blending at the same time.
  // If we allow 0/255 to rail the IIR, then we're no longer protecting
  // against the inrush.
  // We could say if an input is 0/255, lock the other to the same rail (i.e., an off state)
  // However, that would have to be if the value CHANGED and was 0/255, otherwise we'd keep resetting...
  // We also don't want to suddenly stop if changing modes...
  // Maybe we just need an initial allowed-difference window?
  // Could also track the braking mode, and reset when that changes?  Kinda defeats the purpose of toggling though... can't compare betwwen the two.
  // Maybe if the braking mode changes, or if the direction changes, the IIR is reset to current immediately.
  // Effectively, if the PWM-driven pin changes, we reset.

  static eHBridgeIdleMode lastIdle = eIdleUnknown;
  static RelativeDirection lastDirection = eDirectionUnknown;
  // If there was a change in direction or idle mode, assume that the
  // motor speed is consistent?
  // TODO: This still isn't quite correct - starting at 90 degrees would have high in-rush
  if (lastIdle != eIdleUnknown && lastDirection != eDirectionUnknown && (lastIdle != gHBridgeIdleMode || lastDirection != direction)) {
    m1IIR = m1;
    m2IIR = m2;
  } else {
    m1IIR = m1IIR * gMotorIIRWeight + (1.0f - gMotorIIRWeight) * m1;
    m2IIR = m2IIR * gMotorIIRWeight + (1.0f - gMotorIIRWeight) * m2;
  }
  lastDirection = direction;
  lastIdle = gHBridgeIdleMode;

  m1 = (int)m1IIR;
  m2 = (int)m2IIR;
  analogWrite(MOTOR_PIN_1, m1);
  analogWrite(MOTOR_PIN_2, m2);

  // Central dead zone: 0=0
  // For TT motor...
  // PWM Duty ramps:
  //   Running
  //     Has a Min(10), Max(5/8 * 255=160)
  //   Breaking (small-magnitude, 0-10?)
  //     0-Min over short range
  //   Enable true idle, but ramp quickly to functional power
  // PWM trending:
  //   Compare now to short IIR - get rough trend and magnitude up or down
  // PWM Freq TARGET: 1.0 * duty
  //   No adjustments for now...
  // PWM application:
  //   IIR (allows accel to remain slightly trailing / low on the freq)
  //   Quantize: change in steps of 1.2


  // TODO: Do we want an explicit dead zone (very small center window with no motor movement)?
  // TODO: When starting motion, give a little 'boost' and then decay to the actual?
  // TODO: Always allow 0=0 - perhaps have 0 be a true dead zone, and provide a very short ramp to the min PWM, then ramp linearly?
  // NOTE: Seeing a very functional PWM freq around 20-25Hz.  Jerky, but performs well at low power.
  // Delivered power/speed falls off as PWM freq increases, even to only a few 100 Hz.
  // Slow decay mode (high-default, pwm cycles low) improves low-speed performance
  // PWM freq benefits from speed dependence; low speed (20-30 Hz) works well to get started.
  // Lower than that (< 20) is really choppy.  As speed ramps, increase pwn freq to 80-100, or even a bit higher.
  // Min duty can absorb a dead zone.  At low freq (20Hz), dead zone is roughly 0-10 when already stopped.
  // At higher freq, dead zone is larger - 30-60, depending upon frequency.
  // But if min is absolute, there's chatter even when 'stopped'.
  // Propose: Allow ramp to 0, but fast-ramp to the min pwm over just a few steps, then follow linear.
  // i.e., take minimum of fast ramp or linear, and combine that with a freq 20 <= 1.5*pwmLevel <= 80,
  // with an adaptive (short IIR?) when ramping up, to avoid rapid toggling (each change to the freq
  // seems to start a new PWM cycle).  Perhaps also adapt the freq to the power needs - ramping down
  // can rapidly decay the frequency, but ramping up quickly may be biased toward a slower frequency to
  // develop more power until at speed.


  // auto pos = max(0.0, min(180.0, gOrientation.roll + 90.0));  //map(gOrientation.roll, -180, 180, 0 180)
  // if (lastPos != pos) {
  //   myservo.write(pos);
  //   lastPos = pos;
  // }

#endif
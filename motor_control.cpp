#include "motor_control.h"

#include "types.h"
#include "tuning.h"
#include "pins.h"

using namespace EspNowRemote;

bool pitchPidUpdate(float currentPitch, float desiredAngle, float deltaTSec, float& accelOut, DriveParams& dp);
bool velocityPidUpdate(float desiredSpeedNormalized, float deltaTSec, float& pitchTarget, DriveParams& dp);

void initMotors(DriveParams* params) {
  params->idleMode = eBraking;
  // The following three are floats (instead of int) to avoid runtime conversion
  // to float when comparing to gPwmDutyAccumulator / gPwmDutyAppliedMagnitude
  params->deadZone = 1;
  params->pwmMinDuty = 23;
  // 160 is nominal ((2 * 4.2) - 0.7) * (160 / 255) ~= 5V (2x 18650 - diode drop * pwm ratio = rated TT motor voltage)
  // but a bit more oomph helps recovery
  params->pwmMaxDuty = 240; 
  // The IMU tends to shift, and the CoM isn't quite over the axle, so -2.6..-4.0 seems to be the sweet spot
  params->pitchTrim = -2.8f;  
  // The fraction shifted from one motor to the other
  // TODO: Implement PID control for this, driven by encoder input?
  params->yawTrim = 0.0f;
  params->maxThrottleBias = (160.0f / 255.0f);  // Target roughly 5V max throttle (with headroom for correction)
  params->motorFilterWeight = 0.908f;
  params->throttleBias = 0.0f;
  params->steeringBias = 0.0f;

  pinMode(MOTORA_PIN_1, OUTPUT);
  pinMode(MOTORA_PIN_2, OUTPUT);
  pinMode(MOTORB_PIN_1, OUTPUT);
  pinMode(MOTORB_PIN_2, OUTPUT);

  analogWriteFrequency(MOTORA_PIN_1, gPwmFreq);
  analogWriteFrequency(MOTORA_PIN_2, gPwmFreq);
  analogWriteFrequency(MOTORB_PIN_1, gPwmFreq);
  analogWriteFrequency(MOTORB_PIN_2, gPwmFreq);

  analogWriteResolution(MOTORA_PIN_1, PWM_PRECISION);  // scale all pwm output by 2^4 (16)
  analogWriteResolution(MOTORA_PIN_2, PWM_PRECISION);
  analogWriteResolution(MOTORB_PIN_1, PWM_PRECISION);
  analogWriteResolution(MOTORB_PIN_2, PWM_PRECISION);

  // Initializing the motor pins uniformly solves the 'jerk on startup' problem
  if (params->idleMode == eBraking) {
    unsigned int pwmAlwaysOn = 255 * PWM_SCALE_FROM_8BIT;
    analogWrite(MOTORA_PIN_1, pwmAlwaysOn);
    analogWrite(MOTORA_PIN_2, pwmAlwaysOn);
    analogWrite(MOTORB_PIN_1, pwmAlwaysOn);
    analogWrite(MOTORB_PIN_2, pwmAlwaysOn);
  } else {
    analogWrite(MOTORA_PIN_1, 0);
    analogWrite(MOTORA_PIN_2, 0);
    analogWrite(MOTORB_PIN_1, 0);
    analogWrite(MOTORB_PIN_2, 0);
  }
}

// TODO: Thoughts...
// The bot is very short, making any correction movement produce
// rapid angle changes.  Slow the response by raising the center
// of mass.
//
// The motor response also seems to aggressive, ramping rapidly to
// full power.  Not only does this cause wheel slippage, but it makes
// small-scale balance very difficult.  Look into better low-range
// control, possibly remapping the accel data?
//
// TODO: Known problems:
//   * Runaway / persistent velocity - tends to drift after releasing throttle.
//     Why isn't the velocity pid countering lingering duty w/ counter-tilt?
//   * Sometimes, a large change of angle runs away rapidly.  Seems like pitch
//     pid not responding rapidly enough.  Fusion / IMU issue (Kp/Ki)?
void updateMotors(BalanceState* state) {
  static int startupPwmAttenuation = 0;
  unsigned long nowMs = millis();
  auto& dp = state->driveParams;

  // ----------------------------------------
  // Perform updates every 5ms (200/sec)
  // static unsigned long lastUpdate = 0;
  // if (nowMs - lastUpdate < g_update_period)
  //   return;
  // lastUpdate = nowMs;

  // ---------------------------------------
  // Inter-sample time scaling
  // Give the PID feedback accurate time estimates
  // Calculate dT in seconds (the actual time unit is arbitrary, as long as we're consistent)
  static unsigned long lastSampleTimeMs = 0;
  static bool isFirstSample = true;
  float deltaTSec = static_cast<float>(nowMs - lastSampleTimeMs) / 1000.0f;
  lastSampleTimeMs = nowMs;

  // The first sample is used only to set the sample time, so that
  // the next sample (the first real one) can be evaluated with an
  // accurate inter-sample deltaT.
  if (isFirstSample) {
    isFirstSample = false;
    return;
  }

  // ---------------------------------------
  // Diagnostics output trigger (currently disabled)
  static unsigned long lastDiag = 0;
  bool emitDiag = false;
  // emitDiag = nowMs - lastDiag > 200;
  // if (emitDiag)
  //   lastDiag = nowMs;
  if (emitDiag) Serial.printf("------------------------------\n");

  // --------------------------------------
  // Pitch- and Speed-driven PID
  // The desired and average speeds are evaluated, producing a pitch setpoint.
  // The desired and current pitch are evaluated, producing nominal acceleration.
  // The nominal acceleration will later be modified to effect steering.
  // Either PID implementation can indicate a fault to stop the motors after a
  // very short debouncing period.

  // The desired speed is calculated as a normalized (relative) fraction of
  // the configured maximum duty cycle, e.g., [-1.0, 1.0], but is scaled down
  // to leave headroom for balance correction via further acceleration.
  float desiredSpeedNormalized = min(abs(dp.throttleBias), dp.maxThrottleBias);  // dp.throttleBias (unit), dp.maxThrottleBias (1.0)
  if (dp.throttleBias < 0)
    desiredSpeedNormalized *= -1.0;

  float desiredPitchOut = 0.0f;
  bool velocityPidFault = velocityPidUpdate(desiredSpeedNormalized, deltaTSec, desiredPitchOut, dp);
  if (emitDiag) {
    Serial.printf("desiredSpeedNormalized   : %.1f\n", desiredSpeedNormalized);
    Serial.printf("desiredPitchOut          : %.1f\n", desiredPitchOut);
  }

  // The current pitch will be compared to the desired pitch setpoint to determine
  // what acceleration is necessary to achieve that pitch setpoint.
  float pidAccelOut;  // -255..255 nominal
  bool pitchPidFault = pitchPidUpdate(state->imuState.orientation.pitch, desiredPitchOut, deltaTSec, pidAccelOut, dp);
  // The acceleration is constrained to a reasonable range
  float rawAccel = constrain(pidAccelOut, -255.0f, 255.0f);
  if (emitDiag) {
    Serial.printf("pidAccelOut              : %.1f\n", pidAccelOut);
    Serial.printf("rawAccel                 : %.1f\n", rawAccel);
  }

  float constrainedAccel = constrain(rawAccel, (float)-startupPwmAttenuation, (float)startupPwmAttenuation);

  // Upon startup, and after recovery from a fault, enforce a soft-start
  // by 'fading in' the max duty cycle
  if (startupPwmAttenuation < 255)
    ++startupPwmAttenuation;
  if (emitDiag) {
    Serial.printf("constrainedAccel         : %.1f\n", constrainedAccel);
  }

#ifdef ADAPTIVE_FUSION_KI
  // The rate at which the fusion filter corrects integrated gyro data from the gravity (accel) vector
  // is inversely proportional to the time-weighted accel average.  That is, gravity correction is
  // preferentially applied when not under the perturbing influence of linear acceleration (fwd/back).
  // TODO: Account for steering's perturbing effects on applied accel (attenuating fwd/back accel,
  // but increasing side-to-side accel).
  static float normalizedAccelMagPeak = 0.0f;
  normalizedAccelMagPeak = std::max(gAccelPeakDecay * normalizedAccelMagPeak, std::abs(constrainedAccel) / 255.0f);
  state->imuParams.kiScale = 1.0f - normalizedAccelMagPeak;
#endif

  // ----------------------------------
  // Convert PID guidance to normalized PWM duty cycle

  // Adjust the speed by the desired relative acceleration, constraining the duty cycle to the PWM limits.
  // Note that this can be negative, to indicate a reversed direction.
  // TODO: Permit brief excusions beyond gPwmMaxDuty (up to 255) for recovery, but trigger
  //       'unsafe' if operating beyond saturation for more than briefly?)
  //       * Split gPwmMaxDuty into soft & hard limits (default to 255?)
  //       * Maintain a 'cap' reservoir
  //       * Constrain to cap, reduce cap (decrement or decay) if > soft max
  //       * Regenerate toward hard max + delay when target is less than soft max
  //         * don't just reset - need to provide time for the motor to cool down
  //         * delay scale is based on update frequency, decay mode, and permitted time-at-max (should be small)
  // TODO: Physical limit switches to pull DVR8833 SLEEP low if laying down (in addition to the manual switch)
  //       Similarly, use a pulldown resistor with a GPIO as an explicit enable override to avoid spurious motor
  //       activity on startup.
  gPwmDutyAccumulator = constrain(gPwmDutyAccumulator + constrainedAccel, -dp.pwmMaxDuty, dp.pwmMaxDuty);
  if (emitDiag) {
    Serial.printf("gPwmDutyAccumulator      : %.1f\n", gPwmDutyAccumulator);
  }
  // An initial attempt to fix steering...
  // Nominally, steering behaves like a vehicle (i.e., the vehicle's path,
  // forward or backwards, bends toward the steering direction). In practice,
  // this means that the direction of rotation changes when the vehicle
  // switched between forward and backward motion.
  // However, if we need to 'twitch' opposite the direction of travel to
  // maintain balance, we want to maintain the direction of rotation.
  // Like a multi-point turn in a vehicle, this is accomplished by inverting
  // the steering direction when the direction of travel changes.  But we
  // ONLY want to do that when the direction change is corrective.
  // EXPERIMENT: I think it would be better to not simulate a 4-wheel vehicle....
  // A 2-wheel balance bot is always turning about its center.
  // Always invert steering when reversing directions
  // The challenge is near-balance...
  // To account for the dead zone, we'll also maintain rotation direction if
  // the current speed

  // Optional filter, smoothing the motor speed output.
  // In practice, the weight has been in the 0.9-1.0 range, making this filter have little real effect.
  // TODO: Predictive window-based outlier attenuation, but otherwise allow small variations with no additional latency?
  if (dp.motorFilterWeight <= 0.999f) {
    static float motorIIR = 0.0f;
    motorIIR = (dp.motorFilterWeight * gPwmDutyAccumulator) + (1.0f - dp.motorFilterWeight) * motorIIR;
    gPwmDutyAccumulator = motorIIR;
    if (emitDiag) {
      Serial.printf("gPwmDutyAccumulatorSm    : %.1f\n", gPwmDutyAccumulator);
    }
  }

  // Apply steering and compress the output to eliminate the dead zone
  float unsteeredDutyMagnitude = std::abs(gPwmDutyAccumulator);
  bool isUnsteeredReversed = gPwmDutyAccumulator < 0.0f;
  if (emitDiag) {
    Serial.printf("unsteeredDutyMagnitude   : %.1f\n", unsteeredDutyMagnitude);
    Serial.printf("isUnsteeredReversed      : %s\n", isUnsteeredReversed ? "true" : "false");
  }

  // Dampen the steering input to reserve sufficient capacity in each motor for correction
  // Both the trim & bias are normalized ([-1.0, 1.0])
  float effectiveYawFraction = dp.yawTrim + dp.steeringBias * 0.6f;
  if (isUnsteeredReversed)
    effectiveYawFraction *= -1;
  if (emitDiag) {
    Serial.printf("effectiveYawFraction     : %.1f\n", effectiveYawFraction);
  }

  // Steering controls 50% of active range at rest, reduced by 75% at full speed
  // Adaptive steering *reduces* steering influence as speed increases
  // (e.g., to prevent excessively sharp turns at speed)
  float fractionOfMaxSpeed = unsteeredDutyMagnitude / dp.pwmMaxDuty;
  float maxSteeringDelta = ((dp.pwmMaxDuty - dp.pwmMinDuty) * 0.50f) * (1.0f - fractionOfMaxSpeed * 0.75f);
  float appliedSteeringDelta = effectiveYawFraction * maxSteeringDelta;
  // If the steering would put either motor over the max driven duty, back both motors off by the overage
  float steeringOffset = -min(appliedSteeringDelta, max(0.0f, unsteeredDutyMagnitude + appliedSteeringDelta - dp.pwmMaxDuty * dp.maxThrottleBias));
  if (emitDiag) {
    Serial.printf("appliedSteeringDelta     : %.1f\n", appliedSteeringDelta);
    Serial.printf("steeringOffset           : %.1f\n", steeringOffset);
  }

  // Apply the steering, splitting the output into two independent motor channels.
  // We'll work with magnitude (independent of direction), and a flag to indicate each motor's direction.
  // This makes some of the PWM calculations a bit easier.
  // Note that we're NOT changing this math depending upon forward/backward direction...
  // Because we're working with magnitude, for a given yaw direction (e.g., +/clockwise
  // or -/counterclockwise), the same motor increases in speed. This properly simulates
  // a traditional steering approach.
  float unscaledPwmMagnitudeA = min(dp.pwmMaxDuty, unsteeredDutyMagnitude + appliedSteeringDelta + steeringOffset);
  float unscaledPwmMagnitudeB = min(dp.pwmMaxDuty, unsteeredDutyMagnitude - appliedSteeringDelta + steeringOffset);
  if (emitDiag) {
    Serial.printf("unscaledPwmMagnitudeA/B  : %.1f / %.1f\n", unscaledPwmMagnitudeA, unscaledPwmMagnitudeB);
  }

  // The motors' 'reverse' flags are first set by the unsteered direction,
  // but will flip when reconciling (normalizing) a negative speed.
  bool reverseA = isUnsteeredReversed;
  bool reverseB = reverseA;
  if (unscaledPwmMagnitudeA < 0.0f) {
    unscaledPwmMagnitudeA = -unscaledPwmMagnitudeA;
    reverseA = !reverseA;
  }
  if (unscaledPwmMagnitudeB < 0.0f) {
    unscaledPwmMagnitudeB = -unscaledPwmMagnitudeB;
    reverseB = !reverseB;
  }
  if (emitDiag) {
    Serial.printf("reverseA/B               : %s / %s\n", reverseA ? "true" : "false", reverseB ? "true" : "false");
    Serial.printf("unscaledPwmMagnitudeA/B. : %.1f / %.1f\n", unscaledPwmMagnitudeA, unscaledPwmMagnitudeB);
  }

  // Outputs near to zero are entirely shut down. This is the 'dead zone'.
  //   (note that because we're working with magnitude (absolute value), we
  //    don't need to check the negative side of the dead zone)
  // Otherwise, scale the output into the active region
  if (unscaledPwmMagnitudeA <= dp.deadZone)
    unscaledPwmMagnitudeA = 0.0f;
  else
    unscaledPwmMagnitudeA = map(std::abs(unscaledPwmMagnitudeA), dp.deadZone, dp.pwmMaxDuty, dp.pwmMinDuty, dp.pwmMaxDuty);

  if (unscaledPwmMagnitudeB <= dp.deadZone)
    unscaledPwmMagnitudeB = 0.0f;
  else
    unscaledPwmMagnitudeB = map(std::abs(unscaledPwmMagnitudeB), dp.deadZone, dp.pwmMaxDuty, dp.pwmMinDuty, dp.pwmMaxDuty);
  if (emitDiag) {
    Serial.printf("unscaledPwmMagnitudeA/B..: %.1f / %.1f\n", unscaledPwmMagnitudeA, unscaledPwmMagnitudeB);
  }

  // Capture the average applied magnitude, which is used as a proxy for current speed,
  // and is required by the velocity PID for speed control.
  if (reverseA == reverseB)
    gPwmDutyAppliedMagnitude = (unscaledPwmMagnitudeA + unscaledPwmMagnitudeB) / 2.0f;
  else
    gPwmDutyAppliedMagnitude = abs(unscaledPwmMagnitudeA - unscaledPwmMagnitudeB) / 2.0f;
  // if (emitDiag) {
  //   Serial.printf("gPwmDutyAppliedMagnitude : %.1f\n", gPwmDutyAppliedMagnitude);
  // }

  // -----------------------
  // Safety limiter
  // More than 100ms continuously faulted (e.g., at an unsafe angle) will shut down the motor.
  // Upon reactivation, the soft-start is reset

  // Track the contiguous duration over which any fault exists
  static unsigned long anyFaultTimeMs = 0;
  static bool anyFault = false;
  bool wasPidFault = anyFault;
  anyFault = velocityPidFault || pitchPidFault || state->imuState.fault;
  if (anyFault && !wasPidFault) {
    if (velocityPidFault)
      Serial.println("Velocity Fault!");
    else
      Serial.println("Pitch Fault!");
    anyFaultTimeMs = nowMs;
  }

  static bool faultLedState = false;
  if (anyFault && (nowMs - anyFaultTimeMs) > 100) {
    // In case we're experimenting without the motor enabled,
    // the onboard LED is used as a fault indicator.  The
    // transition is placed here (instead of above) to ensure
    // the LED corresponds with the effective fault treatment
    // (i.e., including the 500ms delay).
    if (!faultLedState) {
      digitalWrite(LED_PIN, HIGH);
      faultLedState = true;
    }

    // Reset a bunch of things when recovering from a fault
    gPwmDutyAccumulator = 0.0f;
    gPwmDutyAppliedMagnitude = 0.0f;
    startupPwmAttenuation = 0;
    dp.throttleBias = 0.0f;
    dp.steeringBias = 0.0f;
    unscaledPwmMagnitudeA = unscaledPwmMagnitudeB = 0.0f;
  } else if (!anyFault && faultLedState) {
    digitalWrite(LED_PIN, LOW);
    faultLedState = false;
  }

  // -----------------------
  // Translate duty to control signals

  // Depending upon direction, the DRV8833 needs different pins driven.
  // We also scale the magnitude to match the higher precision of the PWM duty cycle (e.g., 12 bits instead of 8)
  int ma1 = 0, ma2 = 0;
  int mb1 = 0, mb2 = 0;

  // Scale the Pwm duty to the full 12-bit precision that we've configured on the ESP32.
  // This allows for smoother control, especially in the lower end of the pwoer range.
  float scaledPwmMagnitudeA = unscaledPwmMagnitudeA * PWM_SCALE_FROM_8BIT;
  float scaledPwmMagnitudeB = unscaledPwmMagnitudeB * PWM_SCALE_FROM_8BIT;

  // Quantize for compatibility with analogWrite
  int quantizedMagnitudeA = (int)std::round(scaledPwmMagnitudeA);
  int quantizedMagnitudeB = (int)std::round(scaledPwmMagnitudeB);
  if (emitDiag) {
    Serial.printf("quantizedMagnitudeA/B    : %d, %df\n", quantizedMagnitudeA, quantizedMagnitudeB);
  }

  // Here's where we reverse each motor, if needed.
  if (reverseA) {
    ma2 = quantizedMagnitudeA;
  } else {
    ma1 = quantizedMagnitudeA;
  }

  if (reverseB) {
    mb2 = quantizedMagnitudeB;
  } else {
    mb1 = quantizedMagnitudeB;
  }

  // Adjust for braking (slow-decay) / coasting (fast-decay) drive modes.
  // For fast-decay (coasting), one pin is held low, the other is high at the desired duty.
  // For slow-decay (braking), one pin is held high, the other is low at the desired duty.
  // Because analogWrite specifies the high-side duty cycle, slow-decay mode means we invert both
  // of the duty cycles (i.e., 255 on on to hold it high, and 255-duty on the other, so that it is
  // LOW at the desired duty)
  if (dp.idleMode == eBraking) {
    // To invert the H-Bridge input, invert the levels and also
    // swap the driven IO to maintain direction.
    int m1Temp = ma1;
    ma1 = PWM_MAX - ma2;
    ma2 = PWM_MAX - m1Temp;

    m1Temp = mb1;
    mb1 = PWM_MAX - mb2;
    mb2 = PWM_MAX - m1Temp;
  }

  // Update the motor PWM.
  analogWrite(MOTORA_PIN_1, ma1);
  analogWrite(MOTORA_PIN_2, ma2);
  analogWrite(MOTORB_PIN_1, mb1);
  analogWrite(MOTORB_PIN_2, mb2);
}


// The sign of accelOut matches that of the current pitch /error/
// (i.e., if currentAngle > desiredAngle, the accel is positive)
// thus, from a point of stability, increasing desiredAngle (a + change) will result in a (-) accel.
// That corresponds to the base (wheels) moving backwards to achieve a forward tilt.
// Once the current angle exceeds the desired angle (), the accel switches direction,
// seeking to drive the wheels to chase the body.
bool pitchPidUpdate(float currentPitch, float desiredAngle, float deltaTSec, float& accelOut, DriveParams& dp) {
  // TODO: Map accel based on angle, knowing that small angles need
  // very little correction, but high angles need super-linear adjustment.
  // e.g., the accel could be proportional to cos(errorAngle),
  // or (perhaps more accurately), cos(angle) where vertical is 0

  // PID per-update inputs
  // gPitchTrim shifts the reported angle to a 'true' angle.
  // [-90, -90] generally speaking (pitch decreases after 90 for some reason?)
  float currentAngle = currentPitch - dp.pitchTrim;

  // --------------------------------------
  // Pitch-driven fault detection with hysteresis
  bool pitchPidFault;
  {
    static bool pitchFaultFlag = false;
    if (!pitchFaultFlag)
      pitchPidFault = currentAngle < -50 || currentAngle > 50;
    else
      pitchPidFault = currentAngle < -5 || currentAngle > 5;
    pitchFaultFlag = pitchPidFault;
  }

  // -----------------------------
  // Pitch-driven PID

  // How far off are we?
  // TODO: currentAngleAdjusted is where we could subtract the
  // angle error predicted from the current acceleration.
  // This is sometimes known as 'covariance' adjustment,
  // compensating for the acceleration's impact on the
  // apparent gravity vector.
  // The applied acceleration would need to be estimated from the
  // change in speed over time... we could use a circular buffer
  // with a configurable lookback?
  // The challenge is that the IMU responds to accel changes
  // extremely rapidly, making it difficult to accurately predict
  // the accel perturbation.
  // We could also selectively suppress the IMU accel data if under
  // acceleration, shifting emphasis to the integrated gyro for
  // short-term noisy action.  The danger is that balancing is
  // inherently oscillatory, and so frequently under accel.
  float errorAngle = currentAngle - desiredAngle;

  // P: [-90, 90] typical
  float P = errorAngle;

  // I: ?
  static float errorIntegral = 0.0;

  // Clear the integral when we cross the setpoint (don't carry windup across a stable threshold).
  // Note that this doesn't account for inertia - we might wind up crossing rapidly - this suggests
  // a use for the derivative term.
  static int lastErrorSign = 0;
  int currentErrorSign = std::signbit(errorAngle);
  if (currentErrorSign != lastErrorSign)
    errorIntegral = 0;
  lastErrorSign = currentErrorSign;

  // Only accumulate error when we aren't saturated (or very near to it)
  if (gPwmDutyAppliedMagnitude < (dp.pwmMaxDuty - 1.0f)) {
    errorIntegral += errorAngle * deltaTSec;
  }

  float I = errorIntegral;

  // D=D
  // Identify the rate at which the error is changing
  // As this may be very noisy, a low-pass (short IIR) filtered
  // version is also available.  Maybe take the derivative as the
  // difference between two different IIR periods?
  // Note: the Kalman filter is supposed to predict the current delta,
  // rather than work from historical data - a theoretical improvement
  // at additional complexity.
  static float lastErrorAngle = 0.0f;
  static float errorDeltaPerSecondIIR = 0.0f;
  float errorDeltaPerSecond = (errorAngle - lastErrorAngle) / deltaTSec;
  // IIR; weight the accumulator heavily, and the new value lightly.
  errorDeltaPerSecondIIR = gDIIRWeight * errorDeltaPerSecond + (1.0f - gDIIRWeight) * errorDeltaPerSecondIIR;
  lastErrorAngle = errorAngle;
  float D = errorDeltaPerSecondIIR;
  // { // Debug
  //   static unsigned long lastLogMillis = 0;
  //   if (millis() - lastLogMillis > 50) {
  //     Serial.printf("P:%.2f,I:%.2f,D:%.2f\n", P, I, D);
  //     lastLogMillis = millis();
  //   }
  // }
  accelOut = gPitchPidKp * P + gPitchPidKi * I + gPitchPidKd * D;
  return pitchPidFault;
}

// Given a desired speed, guide the pitch.
// If we need to speed up, lean into the appropriate direction
// If we need to slow down, lean away from the direction of travel
bool velocityPidUpdate(float desiredSpeedNormalized, float deltaTSec, float& pitchTarget, DriveParams& dp) {
  // We could look at gPwmDutyAccumulator directly, but we can disregard
  // the deadzone and power scaling by deriving an effective velocity
  // from the applied magnitude relative to the scaled power range.
  // Constrain values lower than gPwmMinDuty to be equivalent to 0.
  // gPwmDutyAppliedMagnitude is pre-adjusted to account for the net impact of steering.
  float currentVelocityNormalized = max(0.0f, (gPwmDutyAppliedMagnitude - dp.pwmMinDuty) / (dp.pwmMaxDuty - dp.pwmMinDuty));

  // Check for out-of-bounds velocity (shouldn't happen?)
  bool velocityPidFault = currentVelocityNormalized < -0.01f || currentVelocityNormalized > 1.01f;

  // Apply the current direction of travel to the normalized magnitude
  if (gPwmDutyAccumulator < 0.0f)
    currentVelocityNormalized *= -1.0f;

  // Smooth the current velocity
  // NOTE: This smoothing (via gSpeedIIRWeight) plays heavily into the ability to control speed.
  // While we want to smooth the current velocity to ignore jittering, we also need the smoothed
  // velocity to respond fast enough to prevent the speed from accelerating to the point where
  // we no longer have enough corrective capacity to balance.  Similarly, we don't want the memory
  // of fast movement to bias the speed during low-throttle input.
  static float smoothedVelocityNormalized = 0.0f;
  smoothedVelocityNormalized = gSpeedIIRWeight * currentVelocityNormalized + (1.0f - gSpeedIIRWeight) * smoothedVelocityNormalized;

  // The subsequent pitch controller produces accel matching the sign of the current pitch /error/ (opposite to control changes)
  // At this point, a currentVelocityNormalized > 0 is travelling in the same direction as a positive pitch would produce.
  // To counteract drift, we would want to target a pitch opposite in sign of currentVelocityNormalized.
  // So, with a velocity target of 0 and a positive currentVelocityNormalized, the error is positive, so we want a
  // negative (inverted) pitch output, which produces positive accel (wheels passing the body) to result in the
  // desired negative pitch, which will require a reversal in direction to maintain.

  // Calculate current error from target
  // This has an absolute magnitude cap of 2.0
  // Note that this compares the time-averaged velocity to the instantaneous target
  float velocityErrorNormalized = smoothedVelocityNormalized - desiredSpeedNormalized;

  // Proportional feedback
  float P = velocityErrorNormalized;

  // Integral feedback with anti-windup
  static float errorIntegral = 0.0f;

  // Clear the integral when crossing the setpoint.
  static int lastErrorSign = 0;
  int currentErrorSign = std::signbit(velocityErrorNormalized);
  if (currentErrorSign != lastErrorSign)
    errorIntegral = 0;
  lastErrorSign = currentErrorSign;

  // Anti-windup: Only integrate error when we have corrective capacity
  // Stop accumulating integral when:
  // 1. Velocity exceeds desired (error is positive, meaning we're going faster than commanded)
  // 2. Motor is saturated at dp.maxThrottleBias (no room for further commanded acceleration)
  float curVelMag = std::abs(currentVelocityNormalized);
  bool isSaturated = curVelMag >= dp.maxThrottleBias;
  bool isExceedingDesiredSpeed = velocityErrorNormalized > 0.001f;

  if (curVelMag > 0.01f && !isSaturated && !isExceedingDesiredSpeed) {
    // velocityErrorNormalized is unitless, measuring the raw delta between normalized current and target velocity.
    // The normalized velocity ranges across [-1.0, 1.0], and we want integration to be reasonably
    // agnostic to the update period.  So... we scale the velocity error by deltaT so that we're
    // integrating one velocityErrorNormalized per second.
    errorIntegral += velocityErrorNormalized * deltaTSec;
  } else if (isSaturated || isExceedingDesiredSpeed) {
    // When saturated or overspeeding, gradually decay the integral to prevent it from
    // persisting and causing overshoot on the next cycle.
    // Decay factor: 0.95 means 5% reduction per cycle; at 200Hz, this will decay rapidly
    errorIntegral *= 0.95f;
  }

  // Any scaling applied to the integral would just a scalar adjustment to Ki, so we'll avoid any further
  // scaling of the integrated error.
  float I = errorIntegral;

  // Derivative feedback
  static float lastErrorVelocity = 0.0f;
  static float errorDeltaPerSecondIIR = 0.0f;
  float errorDeltaPerSecond = (velocityErrorNormalized - lastErrorVelocity) / deltaTSec;
  // IIR; Smooth the delta over multiple readings
  errorDeltaPerSecondIIR = gVelocityDIIRWeight * errorDeltaPerSecond + (1.0f - gVelocityDIIRWeight) * errorDeltaPerSecondIIR;
  lastErrorVelocity = velocityErrorNormalized;
  float D = errorDeltaPerSecondIIR;

  pitchTarget = (gVelocityPidKp * P + gVelocityPidKi * I + gVelocityPidKd * D);
  if (gInvertVelocityPid)
    pitchTarget = -pitchTarget;
#ifdef SERIAL_DIAG
  Serial.printf("SpeedBias:%.2f,PitchTgt:%.3f\n", desiredSpeedNormalized, pitchTarget);
#endif
  return velocityPidFault;
}



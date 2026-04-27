
// ============================================================
// Centerfuge Simple — Motor Controller
// Controls a dual H-bridge motor driver via PWM using a
// potentiometer for speed input.  Includes safety features:
//   - Startup arm lock (knob must be at zero before motor runs)
//   - 15-second continuous-run timeout
//   - Low-speed cutoff (avoids noisy low-duty operation)
//   - Startup kick (brief boost to overcome static friction)
//   - Slew-rate limiting (smooth accel and decel)
//   - Input noise filtering (exponential moving average)
// ============================================================

// ---- Pin assignments ----------------------------------------
const int POT_PIN = A1;   // Analog input: speed-control potentiometer

// ---- Safety timing ------------------------------------------
// Maximum time the motor is allowed to run continuously before
// the system shuts PWM off.  Must turn knob to zero to reset.
const unsigned long PWM_MAX_ON_MS = 15000;  // 15 seconds

// ---- Speed thresholds (0–255 PWM scale) ---------------------
// PWM values at or below this are treated as "motor off" for
// the purposes of resetting the run-timer and arming.
const int PWM_OFF_THRESHOLD = 3;

// Anything below this PWM value is forced to zero.  Prevents
// the motor from running in the low-duty region where it makes
// a high-pitched whine without enough torque to spin properly.
const int PWM_LOW_CUTOFF = 85;   // roughly the bottom third of 0–255

// ---- Startup arm lock ---------------------------------------
// The pot ADC reading (0–1023) must be at or below this value
// for the system to consider the knob "at zero".
const int POT_ARM_THRESHOLD = 20;

// How long the knob must stay near zero before the system arms
// and allows the motor to run.
const unsigned long ARM_HOLD_MS = 800;  // 0.8 seconds

// ---- Slew-rate limits ---------------------------------------
// Maximum PWM change per loop iteration (every 10 ms).
// Smaller step-up = gentler acceleration.
// Larger step-down = quicker but still controlled braking.
const int PWM_STEP_UP   = 2;   // ramp-up rate
const int PWM_STEP_DOWN = 5;   // ramp-down rate

// ---- Pot input filter ---------------------------------------
// Exponential moving average coefficient for the pot reading.
// Lower = smoother but slower to respond.  Range: 0.0–1.0.
const float POT_FILTER_ALPHA = 0.15f;

// ---- Startup kick -------------------------------------------
// When the motor first starts from a full stop, PWM is
// temporarily boosted to KICK_PWM to overcome static friction,
// then tapers back down to whatever the knob is set to.
const int KICK_PWM = 220;                   // boost level (0–255)
const unsigned long KICK_DURATION_MS = 300; // how long the kick lasts

// ---- Runtime state variables --------------------------------
unsigned long pwmOnStartMs = 0;  // millis() timestamp when motor last started running
unsigned long kickStartMs  = 0;  // millis() timestamp when the startup kick began
bool kicking      = false;  // true while the startup kick is active
bool pwmTimedOut  = false;  // true once the 15-second run limit is hit
bool systemArmed  = false;  // true once the knob-at-zero arm condition is satisfied
unsigned long armStartMs = 0;  // millis() timestamp when arm hold timer started
float filteredPot = 0.0f;   // smoothed pot ADC value (exponential moving average)
int   appliedPwm  = 0;      // the PWM value currently being written to the motor

// ---- Motor driver pins --------------------------------------
// The motor driver expects PWM on IN1/IN3 and GND on IN2/IN4
// to spin in one direction.  Both motor coils are driven together.
const int IN1 = 3;   // Motor A PWM (must be a PWM-capable pin)
const int IN2 = 5;   // Motor A direction (held LOW for forward)
const int IN3 = 11;  // Motor B PWM (must be a PWM-capable pin)
const int IN4 = 6;   // Motor B direction (held LOW for forward)

// ---- RGB status LED pins ------------------------------------
// Color represents speed: red = slow, green = fast.
// LED_B has no PWM on this pin so it is driven digitally only.
const int LED_R = 10;  // Red channel   (PWM)
const int LED_G = 8;   // Green channel (PWM)
const int LED_B = 9;   // Blue channel  (digital only)

void setup() {
  Serial.begin(9600);  // Open serial port for debug output at 9600 baud

  // Configure all motor and LED pins as outputs so we can write to them.
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // Explicitly drive every output LOW so the motor and LEDs start in a
  // known-off state.  Without this, pins can float to unpredictable
  // voltages between power-on and the first loop() write.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

  // Seed the low-pass filter with the current pot reading so the first
  // loop iteration does not see a false jump from 0 to wherever the knob is.
  filteredPot = (float)analogRead(POT_PIN);
}

void loop() {
  // --- 1. Read and filter the potentiometer ------------------
  // Raw ADC value: 0 (fully CCW) to 1023 (fully CW).
  // constrain() guards against any occasional out-of-range ADC glitch.
  int potRaw = constrain(analogRead(POT_PIN), 0, 1023);

  // Exponential moving average: blends 15% of the new reading into the
  // running average each loop.  Smooths out electrical noise on the pot
  // without adding significant lag.
  filteredPot += ((float)potRaw - filteredPot) * POT_FILTER_ALPHA;

  // Round to nearest integer and clamp to valid ADC range.
  int pot = constrain((int)(filteredPot + 0.5f), 0, 1023);

  // Capture current time once so all time comparisons in this loop
  // use the same reference point.
  unsigned long now = millis();

  // --- 2. Startup arm lock -----------------------------------
  // After power-on or reset the motor is locked out until the user
  // holds the knob near zero for ARM_HOLD_MS.  This prevents the
  // motor from jumping if the knob was left turned up.
  if (!systemArmed) {
    if (pot <= POT_ARM_THRESHOLD) {
      if (armStartMs == 0) {
        armStartMs = now;  // start the hold timer
      }
      else if (now - armStartMs >= ARM_HOLD_MS) {
        systemArmed = true;  // hold satisfied — allow motor to run
      }
    }
    else {
      armStartMs = 0;  // knob moved away — reset hold timer
    }
  }

  // --- 3. Convert pot to PWM target -------------------------
  // map() scales pot (0–1023) to PWM (255→0), reversing direction so
  // turning CW increases speed.  constrain() ensures map() can never
  // produce a value outside the valid analogWrite range even if the
  // ADC returns a fringe value.
  int pwmTarget = constrain(map(pot, 0, 1023, 255, 0), 0, 255);

  // --- 4. Low-speed cutoff ----------------------------------
  // Force PWM to zero when the target is in the bottom third of the
  // range.  The motor makes a high-pitched whine in this region without
  // producing useful speed, so we skip straight to off.
  if (pwmTarget < PWM_LOW_CUTOFF) {
    pwmTarget = 0;
  }

  // --- 5. Arm lock override ---------------------------------
  // If the system has not yet been armed, keep PWM at zero regardless
  // of what the knob says.
  if (!systemArmed) {
    pwmTarget = 0;
  }

  // --- 6. 15-second run timeout -----------------------------
  // Track how long the motor has been running continuously.
  // Once the limit is reached, lock PWM off until the knob is
  // returned to near zero (which also resets pwmTimedOut).
  if (pwmTarget <= PWM_OFF_THRESHOLD) {
    // Knob is at/near zero: reset the run timer and clear the timeout
    // so the motor can run again on the next start.
    pwmOnStartMs = 0;
    pwmTimedOut = false;
  }
  else {
    if (pwmOnStartMs == 0) {
      pwmOnStartMs = now;  // motor just started — begin timing
    }
    else if (!pwmTimedOut && (now - pwmOnStartMs > PWM_MAX_ON_MS)) {
      pwmTimedOut = true;  // time limit exceeded — shut down
    }
  }

  // Apply timeout: force PWM to zero if the run limit was hit.
  if (pwmTimedOut) {
    pwmTarget = 0;
  }

  // --- 7. Startup kick --------------------------------------
  // When appliedPwm is zero and the target rises above zero, the motor
  // is transitioning from stopped to running.  We briefly override
  // pwmTarget with KICK_PWM to get past static friction, then let the
  // slew ramp handle the rest.
  if (pwmTarget > 0 && appliedPwm == 0 && !kicking) {
    kicking = true;
    kickStartMs = now;  // record when the kick started
  }
  if (kicking) {
    if (pwmTarget == 0) {
      // Motor was commanded off during kick — cancel immediately.
      kicking = false;
    }
    else if (now - kickStartMs < KICK_DURATION_MS) {
      // Still within kick window: boost target to at least KICK_PWM.
      pwmTarget = max(pwmTarget, KICK_PWM);
    }
    else {
      // Kick window expired — release and let slew take over.
      kicking = false;
    }
  }

  // --- 8. Slew-rate limiter ---------------------------------
  // Instead of jumping directly to pwmTarget, appliedPwm steps toward
  // it by at most PWM_STEP_UP (accelerating) or PWM_STEP_DOWN
  // (decelerating) per loop iteration, preventing sudden load spikes.
  if (pwmTarget > appliedPwm) {
    appliedPwm += PWM_STEP_UP;
    if (appliedPwm > pwmTarget) {
      appliedPwm = pwmTarget;  // don't overshoot
    }
  }
  else if (pwmTarget < appliedPwm) {
    appliedPwm -= PWM_STEP_DOWN;
    if (appliedPwm < pwmTarget) {
      appliedPwm = pwmTarget;  // don't undershoot
    }
  }
  // Final hard clamp — guarantees analogWrite never receives an
  // out-of-range value regardless of step arithmetic.
  appliedPwm = constrain(appliedPwm, 0, 255);

  // --- 9. Debug serial output --------------------------------
  // Prints key values every loop so you can monitor behavior over USB.
  // Open the Serial Monitor at 9600 baud to see this.
  Serial.print("Pot: ");
  Serial.print(pot);           // filtered pot ADC value (0–1023)
  Serial.print(" PWM: ");
  Serial.print(appliedPwm);    // actual PWM being sent to motor (0–255)
  Serial.print(" Armed: ");
  Serial.print(systemArmed ? "YES" : "NO");   // arm lock status
  Serial.print(" Timeout: ");
  Serial.println(pwmTimedOut ? "ON" : "OFF"); // 15-sec timeout status

  // --- 10. Motor output -------------------------------------
  // Drive both motor channels (A and B) with the same PWM.
  // IN2 and IN4 are held LOW to set a fixed spin direction.
  // Changing IN2/IN4 to HIGH would reverse the motor.
  analogWrite(IN1, appliedPwm);  // Motor A forward PWM
  analogWrite(IN2, 0);           // Motor A direction: forward
  analogWrite(IN3, appliedPwm);  // Motor B forward PWM
  analogWrite(IN4, 0);           // Motor B direction: forward

  // --- 11. RGB LED speed indicator --------------------------
  // Color transitions smoothly from red (slow) to green (fast)
  // based on the actual applied PWM, not the target, so the LED
  // tracks what the motor is really doing during ramps.
  //   appliedPwm = 0   → full red,   no green
  //   appliedPwm = 255 → no red,     full green
  int r = constrain(map(appliedPwm, 0, 255, 255, 0), 0, 255);  // red fades out as speed rises
  int g = constrain(map(appliedPwm, 0, 255, 0, 255), 0, 255);  // green fades in  as speed rises
  int b = 0;  // blue is unused in this mode

  analogWrite(LED_R, r);
  analogWrite(LED_G, g);

  // LED_B pin does not support PWM so it is driven as a digital signal.
  // b is always 0 here so this will always write LOW (blue off).
  digitalWrite(LED_B, (b > 128) ? HIGH : LOW);

  // --- 12. Loop timing --------------------------------------
  // 10 ms delay gives a ~100 Hz loop rate, which is fast enough for
  // smooth PWM updates while keeping CPU load low.
  delay(10);
}
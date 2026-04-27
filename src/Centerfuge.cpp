#include <Arduino.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
const int POT_PIN = A1;      // Potentiometer input

const int MOTOR_IN1 = 5;     // Motor driver input 1 (D5)
const int MOTOR_IN2 = 6;     // Motor driver input 2 (D6)

const int LED_R_PIN = 10;    // RGB LED red pin (D10)
const int LED_G_PIN = 8;     // RGB LED green pin (D8)
const int LED_B_PIN = 9;     // RGB LED blue pin (D9)
const int STROBE_LED_PIN = 2; // Strobe LED pin (D2)

// ============================================================
// CONSTANTS
// ============================================================

// ------------------------------
// Pot / input settings
// ------------------------------
const int POT_SAMPLES = 8;                 // Number of samples to average
const int POT_DEADBAND = 6;                // Ignore tiny pot changes from noise
const int POT_OFF_THRESHOLD = 40;          // Pot below this means "off / idle"
const int POT_ARM_THRESHOLD = 50;          // Must be below this at startup to arm

// ------------------------------
// Motor speed settings
// ------------------------------
const int MIN_ACTIVE_PWM = 120;            // Lowest useful speed once turned on
const int MAX_ACTIVE_PWM = 255;            // Highest allowed speed (full PWM)
const int PWM_RAMP_STEP = 2;               // How fast speed changes each loop

// ------------------------------
// Timers
// ------------------------------
const unsigned long IDLE_SLEEP_TIME_MS = 15000;     // Go to low-power attract mode after idle
const unsigned long SESSION_TIMEOUT_MS = 45000;     // Exhibit runs for max 45 sec per session
const unsigned long FAULT_BLINK_MS = 300;           // Blink speed during fault
const unsigned long ATTRACT_UPDATE_MS = 25;         // Attract mode LED update timing
const unsigned long LOOP_DELAY_ACTIVE_MS = 15;      // Faster updates while running
const unsigned long LOOP_DELAY_IDLE_MS = 60;        // Slower updates while idle
const int STROBE_MIN_RPM = 600;                     // Estimated low running speed for strobe mapping
const int STROBE_MAX_RPM = 900;                     // Estimated high running speed for strobe mapping

// ------------------------------
// Placeholder stall protection
// NOTE:
// Real stall detection is best done with a current sensor
// or encoder. For now this is just a simple placeholder.
// ------------------------------
const unsigned long HIGH_PWM_STALL_TIME_MS = 8000;  // If stuck near max speed too long, fault
const int HIGH_PWM_STALL_THRESHOLD = 250;

// ============================================================
// ENUM FOR SYSTEM STATES
// ============================================================
enum SystemState
{
    STARTUP_LOCKOUT,   // Must see knob near zero before enabling
    ATTRACT_MODE,      // Waiting for user, LED animation
    IDLE_READY,        // Ready and armed, motor off
    RUNNING,           // Motor running normally
    FAULT_STATE        // Motor shut off due to safety / fault
};

// ============================================================
// GLOBAL VARIABLES
// ============================================================
SystemState currentState = STARTUP_LOCKOUT;

int rawPotValue = 0;
int smoothedPotValue = 0;
int lastSmoothedPotValue = 0;

int targetMotorPwm = 0;     // Desired speed from knob
int currentMotorPwm = 0;    // Actual ramped speed sent to motor

int redValue = 0;
int greenValue = 0;
int blueValue = 0;

unsigned long lastUserActivityTime = 0;
unsigned long sessionStartTime = 0;
unsigned long highPwmStartTime = 0;
unsigned long lastFaultBlinkTime = 0;
unsigned long lastAttractUpdateTime = 0;
unsigned long lastStrobeToggleTime = 0;

bool faultLedOn = false;
bool strobeLedOn = false;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

int readSmoothedPot(void);
void updateStateMachine(void);
void updateMotorControl(void);
void updateLedControl(void);
void updateStrobeControl(void);

void setMotorForward(int pwmValue);
void stopMotor(void);

void setRgbColor(int r, int g, int b);
void setSpeedSpectrumColor(int pwmValue);

void runAttractMode(void);
void runFaultBlink(void);

void enterFaultState(void);
bool isPotNearZero(void);
bool userAdjustedKnob(void);

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(9600);

    // Set output pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    pinMode(STROBE_LED_PIN, OUTPUT);

    // Start safe
    stopMotor();
    setRgbColor(0, 0, 0);
    digitalWrite(STROBE_LED_PIN, LOW);

    // Read first pot value
    smoothedPotValue = readSmoothedPot();
    lastSmoothedPotValue = smoothedPotValue;

    lastUserActivityTime = millis();
    sessionStartTime = 0;
    highPwmStartTime = 0;

    // Startup safety:
    // System stays locked out until knob is near zero.
    currentState = STARTUP_LOCKOUT;
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // Read the smoothed knob position every loop
    smoothedPotValue = readSmoothedPot();

    // Run state logic
    updateStateMachine();

    // Update motor output
    updateMotorControl();

    // Update LED output
    updateLedControl();

    // Update strobe timing LED
    updateStrobeControl();

    // Debug output for testing
    Serial.print("State = ");
    Serial.print(currentState);

    Serial.print(" | Pot = ");
    Serial.print(smoothedPotValue);

    Serial.print(" | Target PWM = ");
    Serial.print(targetMotorPwm);

    Serial.print(" | Current PWM = ");
    Serial.print(currentMotorPwm);

    Serial.print(" | RGB = (");
    Serial.print(redValue);
    Serial.print(", ");
    Serial.print(greenValue);
    Serial.print(", ");
    Serial.print(blueValue);
    Serial.println(")");

    // Slow down loop a little for power saving
    if (currentState == RUNNING)
    {
        delay(LOOP_DELAY_ACTIVE_MS);
    }
    else
    {
        delay(LOOP_DELAY_IDLE_MS);
    }
}

// ============================================================
// readSmoothedPot()
// Takes several analog readings and averages them.
// This helps reduce noise from the potentiometer.
// ============================================================

int readSmoothedPot(void)
{
    long total = 0;

    for (int i = 0; i < POT_SAMPLES; i++)
    {
        total += analogRead(POT_PIN);
        delay(2);  // Small delay between reads for stability
    }

    return total / POT_SAMPLES;
}

// ============================================================
// isPotNearZero()
// Used for startup safety and idle detection.
// ============================================================

bool isPotNearZero(void)
{
    return (smoothedPotValue <= POT_ARM_THRESHOLD);
}

// ============================================================
// userAdjustedKnob()
// Returns true if the knob changed enough to count as a user
// action and not just analog noise.
// ============================================================
bool userAdjustedKnob(void)
{
    if (abs(smoothedPotValue - lastSmoothedPotValue) >= POT_DEADBAND)
    {
        lastSmoothedPotValue = smoothedPotValue;
        lastUserActivityTime = millis();
        return true;
    }

    return false;
}

// ============================================================
// updateStateMachine()
// Handles all high-level system behavior.
// ============================================================

void updateStateMachine(void)
{
    // Keep track of real knob movement
    userAdjustedKnob();

    switch (currentState)
    {
        case STARTUP_LOCKOUT:
        {
            // ------------------------------------------------
            // 9. Startup safety
            // The system will not start until the knob is
            // brought near zero first.
            // ------------------------------------------------
            targetMotorPwm = 0;

            if (isPotNearZero())
            {
                currentState = ATTRACT_MODE;
                lastUserActivityTime = millis();
            }

            break;
        }

        case ATTRACT_MODE:
        {
            // ------------------------------------------------
            // 6. Attract mode
            // Show a gentle LED pattern, motor stays off.
            // ------------------------------------------------
            targetMotorPwm = 0;

            // If the user turns the knob above off threshold,
            // start a session.
            if (smoothedPotValue > POT_OFF_THRESHOLD)
            {
                currentState = RUNNING;
                sessionStartTime = millis();
                lastUserActivityTime = millis();
                highPwmStartTime = 0;
            }

            break;
        }

        case IDLE_READY:
        {
            // ------------------------------------------------
            // 1. Idle ready state
            // Ready for user input, motor off.
            // ------------------------------------------------
            targetMotorPwm = 0;

            if (smoothedPotValue > POT_OFF_THRESHOLD)
            {
                currentState = RUNNING;
                sessionStartTime = millis();
                lastUserActivityTime = millis();
                highPwmStartTime = 0;
            }
            else if ((millis() - lastUserActivityTime) >= IDLE_SLEEP_TIME_MS)
            {
                // ------------------------------------------------
                // 1 and 6. After idle time, go back to attract mode
                // ------------------------------------------------
                currentState = ATTRACT_MODE;
            }

            break;
        }

        case RUNNING:
        {
            // ------------------------------------------------
            // 3. Min and max speed limits
            // If knob is too low, stop running.
            // Otherwise map knob to a safe PWM range.
            // ------------------------------------------------
            if (smoothedPotValue <= POT_OFF_THRESHOLD)
            {
                currentState = IDLE_READY;
                targetMotorPwm = 0;
            }
            else
            {
                // Map usable knob range to limited PWM range
                targetMotorPwm = map(smoothedPotValue,
                                     POT_OFF_THRESHOLD,
                                     1023,
                                     MIN_ACTIVE_PWM,
                                     MAX_ACTIVE_PWM);

                // Clamp it just in case
                if (targetMotorPwm < MIN_ACTIVE_PWM)
                {
                    targetMotorPwm = MIN_ACTIVE_PWM;
                }

                if (targetMotorPwm > MAX_ACTIVE_PWM)
                {
                    targetMotorPwm = MAX_ACTIVE_PWM;
                }
            }

            // ------------------------------------------------
            // 5. Auto stop after a session timeout
            // ------------------------------------------------
            if ((millis() - sessionStartTime) >= SESSION_TIMEOUT_MS)
            {
                currentState = IDLE_READY;
                targetMotorPwm = 0;
            }

            // ------------------------------------------------
            // 4. Placeholder jam / stall protection
            // Not perfect. Just a simple starting point.
            // If running near max speed for too long,
            // enter fault state.
            // ------------------------------------------------
            if (targetMotorPwm >= HIGH_PWM_STALL_THRESHOLD)
            {
                if (highPwmStartTime == 0)
                {
                    highPwmStartTime = millis();
                }
                else if ((millis() - highPwmStartTime) >= HIGH_PWM_STALL_TIME_MS)
                {
                    enterFaultState();
                }
            }
            else
            {
                highPwmStartTime = 0;
            }

            break;
        }

        case FAULT_STATE:
        {
            // ------------------------------------------------
            // 10. Emergency shutdown / fault state
            // Motor stays off until knob is brought near zero.
            // ------------------------------------------------
            targetMotorPwm = 0;

            if (isPotNearZero())
            {
                currentState = ATTRACT_MODE;
                lastUserActivityTime = millis();
                faultLedOn = false;
            }

            break;
        }

        default:
        {
            enterFaultState();
            break;
        }
    }
}

// ============================================================
// updateMotorControl()
// Handles soft start and soft stop by slowly changing the
// current motor PWM toward the target PWM.
// ============================================================
void updateMotorControl(void)
{
    // ------------------------------------------------
    // 2. Soft start and soft stop
    // ------------------------------------------------
    if (currentMotorPwm < targetMotorPwm)
    {
        currentMotorPwm += PWM_RAMP_STEP;

        if (currentMotorPwm > targetMotorPwm)
        {
            currentMotorPwm = targetMotorPwm;
        }
    }
    else if (currentMotorPwm > targetMotorPwm)
    {
        currentMotorPwm -= PWM_RAMP_STEP;

        if (currentMotorPwm < targetMotorPwm)
        {
            currentMotorPwm = targetMotorPwm;
        }
    }

    // Output motor command
    if (currentState == RUNNING && currentMotorPwm > 0)
    {
        setMotorForward(currentMotorPwm);
    }
    else
    {
        stopMotor();
    }
}

// ============================================================
// updateLedControl()
// Handles RGB LED behavior based on system state.
// ============================================================
void updateLedControl(void)
{
    switch (currentState)
    {
        case STARTUP_LOCKOUT:
        {
            // White-ish / pale blue to show "reset knob"
            setRgbColor(80, 80, 120);
            break;
        }

        case ATTRACT_MODE:
        {
            // ------------------------------------------------
            // 6. Attract mode LED pattern
            // ------------------------------------------------
            runAttractMode();
            break;
        }

        case IDLE_READY:
        {
            // ------------------------------------------------
            // 7. LED meaning by state
            // Solid white means ready
            // ------------------------------------------------
            setRgbColor(120, 120, 120);
            break;
        }

        case RUNNING:
        {
            // ------------------------------------------------
            // 7 and 8.
            // Smooth spectrum by speed while running
            // ------------------------------------------------
            setSpeedSpectrumColor(currentMotorPwm);
            break;
        }

        case FAULT_STATE:
        {
            // ------------------------------------------------
            // 7 and 10.
            // Blink red during fault
            // ------------------------------------------------
            runFaultBlink();
            break;
        }

        default:
        {
            setRgbColor(0, 0, 0);
            break;
        }
    }
}

// ============================================================
// setMotorForward()
// Drives the motor in one direction only.
// NOTE:
// This assumes your driver works with IN1 PWM and IN2 LOW.
// If your board is wired differently, we will need to adjust.
// ============================================================
void setMotorForward(int pwmValue)
{
    analogWrite(MOTOR_IN1, pwmValue);
    analogWrite(MOTOR_IN2, 0);
}

// ============================================================
// stopMotor()
// Turns the motor fully off.
// ============================================================
void stopMotor(void)
{
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
}

// ============================================================
// setRgbColor()
// Sends values to the RGB LED.
// NOTE:
// This assumes a common cathode RGB LED.
// If yours is common anode, invert the values later.
// ============================================================


void setRgbColor(int r, int g, int b)
{
    redValue = r;
    greenValue = g;
    blueValue = b;

    analogWrite(LED_R_PIN, redValue);
    analogWrite(LED_G_PIN, greenValue);
    analogWrite(LED_B_PIN, blueValue);
}

// ============================================================
// setSpeedSpectrumColor()
// Smooth spectrum:
// low speed  = blue
// then cyan
// then green
// then yellow
// then red
// ============================================================


void setSpeedSpectrumColor(int pwmValue)
{
    int r = 0;
    int g = 0;
    int b = 0;

    // Convert current PWM from safe running range
    // into a 0 to 255 color scale
    int colorPosition = map(pwmValue, MIN_ACTIVE_PWM, MAX_ACTIVE_PWM, 0, 255);

    if (colorPosition < 0)
    {
        colorPosition = 0;
    }

    if (colorPosition > 255)
    {
        colorPosition = 255;
    }

    if (colorPosition <= 63)
    {
        // Blue -> Cyan
        r = 0;
        g = map(colorPosition, 0, 63, 0, 255);
        b = 255;
    }
    else if (colorPosition <= 127)
    {
        // Cyan -> Green
        r = 0;
        g = 255;
        b = map(colorPosition, 64, 127, 255, 0);
    }
    else if (colorPosition <= 191)
    {
        // Green -> Yellow
        r = map(colorPosition, 128, 191, 0, 255);
        g = 255;
        b = 0;
    }
    else
    {
        // Yellow -> Red
        r = 255;
        g = map(colorPosition, 192, 255, 255, 0);
        b = 0;
    }

    setRgbColor(r, g, b);
}

// ============================================================
// runAttractMode()
// Gentle pulsing / color shift to draw attention.
// Motor remains off.
// ============================================================


void runAttractMode(void)
{
    if ((millis() - lastAttractUpdateTime) >= ATTRACT_UPDATE_MS)
    {
        lastAttractUpdateTime = millis();

        // A simple repeating triangle-style effect using millis
        int phase = (millis() / 10) % 512;

        int brightness = 0;

        if (phase < 256)
        {
            brightness = phase;
        }
        else
        {
            brightness = 511 - phase;
        }

        // Soft blue / purple attract color
        int r = brightness / 4;
        int g = 0;
        int b = brightness / 2;

        setRgbColor(r, g, b);
    }
}

// ============================================================
// runFaultBlink()
// Blinks red while in FAULT_STATE.
// ============================================================

void runFaultBlink(void)
{
    if ((millis() - lastFaultBlinkTime) >= FAULT_BLINK_MS)
    {
        lastFaultBlinkTime = millis();
        faultLedOn = !faultLedOn;

        if (faultLedOn)
        {
            setRgbColor(255, 0, 0);
        }
        else
        {
            setRgbColor(0, 0, 0);
        }
    }
}

// ============================================================
// enterFaultState()
// Forces motor off and enters fault mode.
// ============================================================
void enterFaultState(void)
{
    currentState = FAULT_STATE;
    targetMotorPwm = 0;
    currentMotorPwm = 0;
    stopMotor();
}

// ============================================================
// updateStrobeControl()
// Flashes a dedicated strobe LED at a fixed cadence while RUNNING.
// Outside RUNNING it is forced off.
// ============================================================
void updateStrobeControl(void)
{
    if (currentState == RUNNING)
    {
        unsigned long now = millis();
        int estimatedRpm = map(currentMotorPwm,
                               MIN_ACTIVE_PWM,
                               MAX_ACTIVE_PWM,
                               STROBE_MIN_RPM,
                               STROBE_MAX_RPM);

        if (estimatedRpm < STROBE_MIN_RPM)
        {
            estimatedRpm = STROBE_MIN_RPM;
        }

        if (estimatedRpm > STROBE_MAX_RPM)
        {
            estimatedRpm = STROBE_MAX_RPM;
        }

        // Half-period in ms. Since Hz = RPM / 60:
        // halfPeriodMs = 1000 / (2 * Hz) = 30000 / RPM.
        unsigned long strobeIntervalMs = 30000UL / (unsigned long)estimatedRpm;

        if ((now - lastStrobeToggleTime) >= strobeIntervalMs)
        {
            // Keep cadence even by stepping in fixed intervals.
            lastStrobeToggleTime += strobeIntervalMs;

            // If the loop was delayed a long time, resync cleanly.
            if ((now - lastStrobeToggleTime) >= strobeIntervalMs)
            {
                lastStrobeToggleTime = now;
            }

            strobeLedOn = !strobeLedOn;
            digitalWrite(STROBE_LED_PIN, strobeLedOn ? HIGH : LOW);
        }
    }
    else
    {
        strobeLedOn = false;
        lastStrobeToggleTime = millis();
        digitalWrite(STROBE_LED_PIN, LOW);
    }
}
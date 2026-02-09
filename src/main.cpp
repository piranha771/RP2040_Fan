#include <Arduino.h>

// ### PIN WIRING
static const uint8_t TEMP1_PIN = 29;
static const uint8_t TEMP2_PIN = 28;
static const uint8_t MOBO_PWM1_IN_PIN = 27;
static const uint8_t MOBO_PWM2_IN_PIN = 26;
static const uint8_t FAN1_PWM_OUT_PIN = 15;
static const uint8_t FAN2_PWM_OUT_PIN = 14;
// ########

// Tunables for 25kHz PWM (40us period)
static const uint32_t GLITCH_REJECT_US   = 1;    // reject edges closer than this
static const uint32_t MIN_PERIOD_US      = 25;   // ~40kHz upper bound (sanity)
static const uint32_t MAX_PERIOD_US      = 80;   // ~12.5kHz lower bound (sanity)
static const uint32_t STUCK_TIMEOUT_US   = 200;  // 5 periods => treat as stuck

typedef struct {
    // raw edge timing
    volatile uint32_t last_edge_us;
    volatile bool     last_level;

    // tracking the current candidate cycle
    volatile uint32_t rise_us;        // time of last accepted rising edge
    volatile uint32_t fall_us;        // time of last accepted falling edge
    volatile bool     saw_fall;        // did we see a fall after the last rise?

    // last completely valid cycle
    volatile uint32_t good_period_us; // last valid period (rise->rise)
    volatile uint32_t good_high_us;   // last valid high time (rise->fall)
    volatile uint32_t last_good_us;   // timestamp when good_* was updated (i.e., a full valid cycle completed)

    volatile uint32_t rejected_edges; // debug counter
} pwm_meas_t;

static pwm_meas_t pwm1, pwm2;

#define FAN_FREQ_HZ 22720
#define FILTER_ALPHA 0.1

#define UPDATE_RATE_HZ 100
#define UPDATE_DELAY_MS 1000 / UPDATE_RATE_HZ

#define PWM_MAX_VAL 255

uint16_t mobo_fan = 0;
uint8_t last_out_duty = 0;

uint32_t tLastPrint = 0;
uint32_t tLastUpdate = 0;

static inline void on_pwm_change(pwm_meas_t *p, uint32_t now_us, bool level)
{
    // 1) Glitch/duplicate reject based on edge spacing
    uint32_t dt_edge = now_us - p->last_edge_us;
    if (p->last_edge_us != 0 && dt_edge < GLITCH_REJECT_US) {
        p->rejected_edges++;
        return;
    }

    // 2) If we somehow get "same level again" (can happen with ISR latency + bounce), ignore
    if (p->last_edge_us != 0 && level == p->last_level) {
        p->rejected_edges++;
        // Still update last_edge_us to avoid a storm counting as "old"
        p->last_edge_us = now_us;
        return;
    }

    p->last_edge_us = now_us;
    p->last_level   = level;

    if (level) {
        // =========================
        // Rising edge
        // =========================
        uint32_t prev_rise = p->rise_us;

        // Accept this rise as the new reference (but only after we possibly close a valid cycle)
        if (prev_rise != 0 && p->saw_fall) {
            // Candidate completed cycle: rise(prev) -> fall -> rise(now)
            uint32_t period = now_us - prev_rise;
            uint32_t high   = p->fall_us - prev_rise;  // safe because saw_fall implies fall_us was set after prev_rise

            // Validate cycle
            if (period >= MIN_PERIOD_US && period <= MAX_PERIOD_US && high <= period) {
                // Full valid cycle. This is the ONLY place we update "good"
                p->good_period_us = period;
                p->good_high_us   = high;
                p->last_good_us   = now_us;
            } else {
                p->rejected_edges++;
            }
        }

        // Start new cycle candidate
        p->rise_us  = now_us;
        p->saw_fall = false;
    }
    else {
        // =========================
        // Falling edge
        // =========================
        uint32_t r = p->rise_us;
        if (r == 0) {
            // fall before any rise: ignore
            p->rejected_edges++;
            return;
        }

        uint32_t high = now_us - r;
        // Basic sanity: high must be non-negative and not insane.
        // Note: For very small duty cycles, high can be tiny; GLITCH_REJECT_US protects bounce.
        if (high >= 0) {
            p->fall_us  = now_us;
            p->saw_fall = true;
        } else {
            p->rejected_edges++;
        }
    }
}
void on_pwm1_change() {
    on_pwm_change(&pwm1, micros(), digitalReadFast(MOBO_PWM1_IN_PIN));
}

void on_pwm2_change() {
   on_pwm_change(&pwm2, micros(), digitalReadFast(MOBO_PWM2_IN_PIN));
}

uint16_t pwm_get_fan_command_0_10000(pwm_meas_t *p, uint8_t pin, uint8_t &return_code, uint16_t value_when_invalid)
{
    uint32_t last_good, period, high, now;
    bool level;

    noInterrupts();
    now       = micros();
    last_good = p->last_good_us;
    period    = p->good_period_us;
    high      = p->good_high_us;
    interrupts();

    uint32_t age = now - last_good;

    // Evaluate "stuck" using current pin level
    if (last_good == 0 || age > STUCK_TIMEOUT_US) {
        // Stuck: decide by actual pin level (or cache last_level if you prefer)
        level = digitalReadFast(pin);
        return_code = 1;
        return level ? 10000u : 0u;
    }

    uint16_t duty = (uint16_t)((high * 10000u) / period);
    return_code = 4;
    return duty;
}

void setup() 
{
  analogWriteFreq(FAN_FREQ_HZ);
  analogWriteResolution(8);
  analogReadResolution(12);
  pinMode(TEMP1_PIN, INPUT);
  pinMode(TEMP2_PIN, INPUT);
  pinMode(MOBO_PWM1_IN_PIN, INPUT);
  pinMode(MOBO_PWM2_IN_PIN, INPUT);
  pinMode(FAN1_PWM_OUT_PIN, OUTPUT);
  pinMode(FAN2_PWM_OUT_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(MOBO_PWM1_IN_PIN), on_pwm1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOBO_PWM2_IN_PIN), on_pwm2_change, CHANGE);

  Serial.begin(115200);
}



float transformMoboDuty(uint16_t mobo_pwm)
{
    return max(0, min(10000, mobo_pwm * (10.0f/6.0f)));
}


int32_t show = 0;
int32_t oob = 0;
float alpha = 0.98f;
void loop() {
    uint32_t now = micros();
    uint8_t return1_code = 0;
    uint8_t return2_code = 0;
    uint16_t mobo_fan1 = pwm_get_fan_command_0_10000(&pwm1, MOBO_PWM1_IN_PIN, return1_code, mobo_fan);
    uint16_t mobo_fan2 = pwm_get_fan_command_0_10000(&pwm2, MOBO_PWM2_IN_PIN, return2_code, mobo_fan);

    if (return1_code == 3 || return2_code == 3) oob = 1;

    mobo_fan = mobo_fan * alpha + ((1.0f - alpha) * max(mobo_fan1, mobo_fan2));

    last_out_duty = transformMoboDuty(mobo_fan) / 10000.0f * 255.0f;

    analogWrite(FAN1_PWM_OUT_PIN, 255 - last_out_duty);
    analogWrite(FAN2_PWM_OUT_PIN, 255 - last_out_duty);

    tLastUpdate = millis();

    if (millis() - tLastPrint > 100)
    {
        Serial.print("[");
        Serial.print(show++);
        Serial.print("] IN: ");
        Serial.print(mobo_fan1 / 100.0f);
        Serial.print(" {");
        Serial.print(return1_code);
        Serial.print("}");
        Serial.print(" IN2: ");
        Serial.print(mobo_fan2 / 100.0f);
        Serial.print(" {");
        Serial.print(return2_code);
        Serial.print("}");
        Serial.print(" OOB: ");
        Serial.print(oob);
        Serial.print("    \r");
        tLastPrint = millis();
    }
}

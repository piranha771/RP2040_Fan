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
static const uint32_t GLITCH_REJECT_US   = 3;    // reject edges closer than this
static const uint32_t MIN_PERIOD_US      = 25;   // ~40kHz upper bound (sanity)
static const uint32_t MAX_PERIOD_US      = 80;   // ~12.5kHz lower bound (sanity)
static const uint32_t STUCK_TIMEOUT_US   = 200;  // 5 periods => treat as stuck

typedef struct {
    volatile uint32_t last_rise_us;
    volatile uint32_t period_us;
    volatile uint32_t high_us;
    volatile uint32_t last_edge_us;   // any edge
    volatile uint8_t  is_valid_period;
} pwm_meas_t;

static pwm_meas_t pwm1, pwm2;

#define FAN_FREQ_HZ 22720
#define FILTER_ALPHA 0.1

#define UPDATE_RATE_HZ 100
#define UPDATE_DELAY_MS 1000 / UPDATE_RATE_HZ

#define PWM_MAX_VAL 255

float mobo_fan = 0;
uint8_t last_out_duty = 0;

uint32_t tLastPrint = 0;
uint32_t tLastUpdate = 0;

static inline void on_pwm_change(pwm_meas_t *p, uint32_t now_us, bool level)
{
    p->last_edge_us = now_us;

    if (level) {
        // Rising edge
        uint32_t dt = (uint32_t)(now_us - p->last_rise_us);
        if (p->last_rise_us != 0) {
            // Glitch reject + sanity band
            if (dt >= GLITCH_REJECT_US && dt >= MIN_PERIOD_US && dt <= MAX_PERIOD_US) {
                p->period_us = dt;
                p->is_valid_period = 1;
            }
        }
        p->last_rise_us = now_us;
    } else {
        // Falling edge
        if (p->last_rise_us != 0) {
            uint32_t ht = (uint32_t)(now_us - p->last_rise_us);
            if (ht >= GLITCH_REJECT_US) {
                p->high_us = ht;
            }
        }
    }
}

void on_pwm1_change() {
    on_pwm_change(&pwm1, micros(), digitalReadFast(MOBO_PWM1_IN_PIN));
}

void on_pwm2_change() {
   on_pwm_change(&pwm2, micros(), digitalReadFast(MOBO_PWM2_IN_PIN));
}

uint16_t pwm_get_fan_command_0_10000(pwm_meas_t *p, uint8_t pin)
{
    uint32_t last_edge, period, high;
    uint8_t  is_valid_period;

    noInterrupts();
    uint32_t now_us = micros();
    last_edge   = p->last_edge_us;
    period      = p->period_us;
    high        = p->high_us;
    is_valid_period = p->is_valid_period;
    interrupts();

    if (!is_valid_period || period == 0) {
        return 0; // or "keep last value" in caller
    }

    uint32_t age = (uint32_t)(now_us - last_edge);

    // Evaluate "stuck" using current pin level (read outside the critical section)
    if (age > STUCK_TIMEOUT_US) {
        bool level = digitalRead(pin);
        uint16_t duty = level ? 10000 : 0;
        return duty;
    }


    if (high > period) high = period;

    uint16_t duty = (uint16_t)((high * 10000u) / period);
    return duty;
}

void setup() 
{
  analogWriteFreq(FAN_FREQ_HZ);
  analogWriteResolution(8);
  analogReadResolution(12);
  pinMode(TEMP1_PIN, INPUT);
  pinMode(TEMP2_PIN, INPUT);
  pinMode(MOBO_PWM1_IN_PIN, INPUT_PULLUP);
  pinMode(MOBO_PWM2_IN_PIN, INPUT_PULLUP);
  pinMode(FAN1_PWM_OUT_PIN, OUTPUT);
  pinMode(FAN2_PWM_OUT_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(MOBO_PWM1_IN_PIN), on_pwm1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOBO_PWM2_IN_PIN), on_pwm2_change, CHANGE);

  Serial.begin(115200);
}

int32_t show = 0;

float transformMoboDuty(float mobo_pwm)
{
    return max(0, min(10000, mobo_pwm));
}

void loop() {
    uint32_t now = micros();
    uint16_t mobo_fan1 = 10000 - pwm_get_fan_command_0_10000(&pwm1, MOBO_PWM1_IN_PIN);
    uint16_t mobo_fan2 = 10000 - pwm_get_fan_command_0_10000(&pwm2, MOBO_PWM2_IN_PIN);

    if (millis() - tLastUpdate > UPDATE_DELAY_MS)
    {    
        mobo_fan = mobo_fan1 > mobo_fan2 ? mobo_fan1 : mobo_fan2;

        last_out_duty = transformMoboDuty(mobo_fan) / 10000.0f * 255.0f;

        analogWrite(FAN1_PWM_OUT_PIN, last_out_duty);
        analogWrite(FAN2_PWM_OUT_PIN, last_out_duty);

        tLastUpdate = millis();
    }

    if (millis() - tLastPrint > 100)
    {
        Serial.print("[");
        Serial.print(show++);
        Serial.print("] IN: ");
        Serial.print(mobo_fan1 / 100.0f);
        Serial.print(" IN2: ");
        Serial.print(mobo_fan2 / 100.0f);
        Serial.print("    OUT: ");
        Serial.print(((255.0f - last_out_duty) / 255.0f) * 100.0f);
        Serial.print("%    \r");
        tLastPrint = millis();
    }
}

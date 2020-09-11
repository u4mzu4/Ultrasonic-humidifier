#include "driver/pcnt.h"

#define BUT_PIN 34

#define PWM_PIN  12
#define PWM_FREQ 25000
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 7

#define PCNT_INPUT_PIN  36
#define PCNT_FILT_VAL   1023
#define PCNT_L_LIM_VAL  0
#define PCNT_H_LIM_VAL  14400


void PCNT_config()
{
  pinMode(PCNT_INPUT_PIN, INPUT);

  pcnt_config_t pcntFreqConfig = { };
  pcntFreqConfig.pulse_gpio_num = PCNT_INPUT_PIN;
  pcntFreqConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcntFreqConfig.lctrl_mode = PCNT_MODE_KEEP;
  pcntFreqConfig.hctrl_mode = PCNT_MODE_KEEP;
  pcntFreqConfig.pos_mode = PCNT_COUNT_INC;
  pcntFreqConfig.neg_mode = PCNT_COUNT_DIS;
  pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;
  pcntFreqConfig.counter_l_lim = PCNT_L_LIM_VAL;
  pcntFreqConfig.unit = PCNT_UNIT_0;
  pcntFreqConfig.channel = PCNT_CHANNEL_0;

  pcnt_unit_config(&pcntFreqConfig);
  pcnt_intr_disable(PCNT_UNIT_0);
  pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILT_VAL);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

void PWM_config()
{
  pinMode(PWM_PIN, OUTPUT);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_CHANNEL, 127);
}

void setup() {
  Serial.begin(115200);
  PCNT_config();
  PWM_config();
  pinMode(BUT_PIN, INPUT);
}

void loop() {
  static unsigned long timer;
  static unsigned short counter = 0;
  static unsigned short dutyCycle = 127;

  if (millis() - timer > 1000)
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &counter);
    pcnt_counter_clear(PCNT_UNIT_0);
    Serial.print("Counter:");
    Serial.println(60*counter/2);
    timer = millis();
  }
  if (!digitalRead(BUT_PIN))
  {
    dutyCycle--;
    ledcWrite(PWM_CHANNEL, dutyCycle);
    delay(100);
  }
  if (dutyCycle < 1)
  {
    dutyCycle = 127;
  }
}

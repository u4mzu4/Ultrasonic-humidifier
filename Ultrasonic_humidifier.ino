#include "driver/pcnt.h"

#define POWER_PIN		4
#define PWM_PIN			12
#define DAC_PIN			25
#define BUT_PIN			34
#define PCNT_INPUT_PIN  36

#define PWM_FREQ		25000
#define PWM_CHANNEL		0
#define PWM_RESOLUTION	7
#define PWM_MAX_VALUE	127 	//2^PWM_RESOLUTION - 1
#define PCNT_FILT_VAL   1023
#define PCNT_L_LIM_VAL  0
#define PCNT_H_LIM_VAL  14400
#define DAC_DEFAULT  	96
#define DAC_STEP	  	38.0	//6000 / (255 - DAC_DEFAULT)
#define DAC_MAX_VALUE  	255		//2^8 - 1
#define RPM_STEP	  	47 		//6000 / PWM_MAX_VALUE
#define MAX_SPEED  		5969 	//PWM_MAX_VALUE * RPM_STEP
#define SETTLING_TIME  	3


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
  ledcWrite(PWM_CHANNEL, PWM_MAX_VALUE);
}

void PWM_Control(unsigned int reqSpeed);
{
	short counter;
	unsigned int measRPM;
	unsigned byte dutyCycle;
	static unsigned int prevSpeed;
	static unsigned byte tunedDutyCycle = PWM_MAX_VALUE;
	static unsigned byte waitForSettling = 0;
	
	pcnt_get_counter_value(PCNT_UNIT_0, &counter);
	measRPM = 30*counter; //60 * counter / 2
	dutyCycle = round(float(PWM_MAX_VALUE)-float(reqSpeed)/float(RPM_STEP));
	
	Serial.print("Blower RPM:");
    Serial.println(measRPM);
    Serial.print("PMW duty:");
    Serial.println(PWM_MAX_VALUE - dutyCycle);
	
	if (abs(reqSpeed-measRPM) < 2*RPM_STEP) //speed OK
	{
		prevSpeed = reqSpeed;
		pcnt_counter_clear(PCNT_UNIT_0);
		return;
	}
	
	if (reqSpeed != prevSpeed) //new PWM needed
	{
		ledcWrite(PWM_CHANNEL, dutyCycle);
		waitForSettling = SETTLING_TIME;
		prevSpeed = reqSpeed;
		tunedDutyCycle = dutyCycle;
		pcnt_counter_clear(PCNT_UNIT_0);
		return;
	}
	
	if (waitForSettling > 0) //waiting for settling
	{
		waitForSettling--;
		pcnt_counter_clear(PCNT_UNIT_0);
		return;
	}

	if (measRPM < reqSpeed) //fine tuning1 needed
	{
		tunedDutyCycle--;
		if (tunedDutyCycle <1)
		{
			tunedDutyCycle = 1;
		}
		if ((dutyCycle-tunedDutyCycle) > 5)
		{
			Serial.println("PMW out of tolerance!");
			return;
		}
		ledcWrite(PWM_CHANNEL, tunedDutyCycle);
		waitForSettling = SETTLING_TIME;
		prevSpeed = reqSpeed;
		pcnt_counter_clear(PCNT_UNIT_0);
		return;
	}
	
	if (measRPM > reqSpeed) //fine tuning2 needed
	{
		tunedDutyCycle++;
		if (tunedDutyCycle > (PWM_MAX_VALUE-1))
		{
			tunedDutyCycle = PWM_MAX_VALUE-1;
		}
		if ((tunedDutyCycle-dutyCycle) > 5)
		{
			Serial.println("PMW out of tolerance!");
			return;
		}
		ledcWrite(PWM_CHANNEL, tunedDutyCycle);
		waitForSettling = SETTLING_TIME;
		prevSpeed = reqSpeed;
		pcnt_counter_clear(PCNT_UNIT_0);
		return;
	}
}

void DAC_Control(unsigned int powerBasedOnRPM);
{
	static unsigned int prevPower = 0;
	unsigned byte dacValue;
	
	if (powerBasedOnRPM = prevPower)
	{
		return;
	}
	else
	{
		dacValue = round(float(DAC_MAX_VALUE) - float(powerBasedOnRPM)/DAC_STEP);
		Serial.print("dacValue:");
    	Serial.println(dacValue);
		dacWrite(DAC_PIN, dacValue);
		prevPower = powerBasedOnRPM();
	}
}

void setup() {
  Serial.begin(115200);
  PCNT_config();
  PWM_config();
  pinMode(BUT_PIN, INPUT);
  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, DAC_MAX_VALUE);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
}

void loop() {
  static unsigned long timer;
  static unsigned int blowerSpeed = 0;

  if (millis() - timer > 1000)
  {
	PWM_Control(blowerSpeed);
	timer = millis();
	DAC_Control(blowerSpeed);
  }
  
  if (!digitalRead(BUT_PIN))
  {
	if (!digitalRead(POWER_PIN))
	{
		digitalWrite(POWER_PIN, HIGH);
		ledcWrite(PWM_CHANNEL, PWM_MAX_VALUE);
    	dacWrite(DAC_PIN, DAC_MAX_VALUE);
	}
	else
	{	
    	blowerSpeed +=  RPM_STEP;
    	if (blowerSpeed > MAX_SPEED)
		{
		blowerSpeed = 0;
		}
    	delay(100);
  	}
  }
}
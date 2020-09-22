/*
  ESP32-EVB development board (https://www.olimex.com/Products/IoT/ESP32/ESP32-EVB/open-source-hardware)
  BME280 sensor
  LM2596 programmable power supply
  Sunon MF50152VX-1L01C-S99 blower
  I2C rotary (https://github.com/Fattoresaimon/I2CEncoderV2.1)
  Blynk service
*/


//Includes
#include "driver/pcnt.h"
#include <Adafruit_BME280.h>
#include <NTPtimeESP.h>
#include <BlynkSimpleEsp32_SSL.h>
#include <i2cEncoderLibV2.h>
#include <credentials.h>

//Enum
enum MAIN_SM {
  INIT    = 0,
  OFF   = 1,
  AUTO    = 2,
  MANUAL  = 3,
  NIGHT   = 4,
  FAILED  = 7,
  FW_UPDATE = 9
};


//Defines
#define PWM_PIN         12
#define DAC_PIN         25
#define RELAY_PIN       32
#define BUT_PIN         34
#define PCNT_INPUT_PIN  36
#define INTERRUPT_PIN   17

#define FALSE           false
#define TRUE            true
#define PWM_FREQ        25000
#define PWM_CHANNEL     0
#define PWM_RESOLUTION  7
#define PWM_MAX_VALUE   127   //2^PWM_RESOLUTION - 1
#define PCNT_FILT_VAL   1023
#define PCNT_L_LIM_VAL  0
#define PCNT_H_LIM_VAL  14400
#define DAC_DEFAULT     96
#define DAC_STEP        38.0  //6000 / (255 - DAC_DEFAULT)
#define DAC_MAX_VALUE   255   //2^8 - 1
#define RPM_STEP        47    //6000 / PWM_MAX_VALUE
#define MAX_SPEED       5969  //PWM_MAX_VALUE * RPM_STEP
#define SETTLING_TIME   3
#define DEF_HUMIDITY    40    //DEFAULT AUTO SETTINGS
#define MIN_HUMIDITY    19    //LOWEST MEASURED HUMIDITY EVER
#define AUTO_TIMER      60000 //1 min
#define NIGHT_BEFORE    7   //NIGHT END
#define NIGHT_AFTER     21    //NIGHT START
#define ENCODER_ADDRESS 0x02
#define TIMEOUT         5000  //5 sec
#define NTPSERVER       "hu.pool.ntp.org"

//Global variables
bool rotaryEvent = FALSE;
bool failSafe = FALSE;
bool isAutoMode;
float actualTemperature;
float actualHumidity;
float actualPressure;
float setHumidity;

//Init services
strDateTime dateTime;
Adafruit_BME280 bme;
NTPtime NTPhu(NTPSERVER);   // Choose server pool as required
//BlynkTimer timer;
WidgetTerminal terminal(V1);
i2cEncoderLibV2 Encoder(ENCODER_ADDRESS);


void ReadBME280()
{
  static float lastvalidTemperature;
  static float lastvalidHumidity;
  static float lastvalidPressure;
  static int bmeErrorcounter = 0;

  bme.takeForcedMeasurement();
  actualTemperature = bme.readTemperature();
  actualHumidity = bme.readHumidity();
  actualPressure = bme.readPressure() / 100.0F;

  if (actualTemperature < 1.0 || actualTemperature > 100.0) {
    actualTemperature = lastvalidTemperature;
    bmeErrorcounter++;
  }
  else {
    lastvalidTemperature = actualTemperature;
    bmeErrorcounter = 0;
  }
  if (actualPressure > 1086.0 || actualPressure < 870.0) {
    actualPressure = lastvalidPressure;
  }
  else {
    lastvalidPressure = actualPressure;
  }
  if (actualHumidity > 99.0) {
    actualHumidity = lastvalidHumidity;
  }
  else {
    lastvalidHumidity = actualHumidity;
  }
  //ErrorManager(BME280_ERROR, bmeErrorcounter, 5);
  //Blynk.virtualWrite(V2, bmeTemperature);
  //Blynk.virtualWrite(V3, actualHumidity);
  //Blynk.virtualWrite(V4, actualPressure);
}

void IRAM_ATTR HandleInterrupt() {
  rotaryEvent = TRUE;
}

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

void PWM_Control(unsigned int reqSpeed)
{
  short counter;
  unsigned int measRPM;
  byte dutyCycle;
  static unsigned int prevSpeed;
  static byte tunedDutyCycle = PWM_MAX_VALUE;
  static byte waitForSettling = 0;

  pcnt_get_counter_value(PCNT_UNIT_0, &counter);
  measRPM = 30 * counter; //60 * counter / 2
  dutyCycle = round(float(PWM_MAX_VALUE) - float(reqSpeed) / float(RPM_STEP));

  Serial.print("Blower RPM:");
  Serial.println(measRPM);
  Serial.print("PMW duty:");
  Serial.println(PWM_MAX_VALUE - dutyCycle);

  if (abs(reqSpeed - measRPM) < 2 * RPM_STEP) //speed OK
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
    if (tunedDutyCycle < 1)
    {
      tunedDutyCycle = 1;
    }
    if ((dutyCycle - tunedDutyCycle) > 5)
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
    if (tunedDutyCycle > (PWM_MAX_VALUE - 1))
    {
      tunedDutyCycle = PWM_MAX_VALUE - 1;
    }
    if ((tunedDutyCycle - dutyCycle) > 5)
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

void Threewire_Control(unsigned int reqSpeed)
{
  byte dutyCycle;
  static byte waitForSettling = 0;
  unsigned int prevSpeed;

  dutyCycle = round(float(PWM_MAX_VALUE) - float(reqSpeed) / float(RPM_STEP));

  Serial.print("PMW duty:");
  Serial.println(PWM_MAX_VALUE - dutyCycle);

  if (reqSpeed != prevSpeed) //new PWM needed
  {
    ledcWrite(PWM_CHANNEL, dutyCycle);
    waitForSettling = SETTLING_TIME;
    prevSpeed = reqSpeed;
    return;
  }

  if (waitForSettling > 0) //waiting for settling
  {
    waitForSettling--;
    return;
  }
}

void DAC_Control(unsigned int powerBasedOnRPM)
{
  static unsigned int prevPower = 0;
  byte dacValue;

  if (powerBasedOnRPM = prevPower)
  {
    return;
  }
  else
  {
    dacValue = round(float(DAC_MAX_VALUE) - float(powerBasedOnRPM) / DAC_STEP);
    Serial.print("dacValue:");
    Serial.println(dacValue);
    dacWrite(DAC_PIN, dacValue);
    prevPower = powerBasedOnRPM;
  }
}

void RotaryCheck()
{
  static MAIN_SM stateMachine = INIT;
  int rotaryPosition;

  switch (stateMachine)
  {
    case INIT:
      {
       Encoder.writeMax((int32_t)6000); /* Set the maximum  */
       Encoder.writeMin((int32_t)0); /* Set the minimum threshold */
       Encoder.writeStep((int32_t)300);
       Encoder.writeCounter((int32_t)0);
       stateMachine = OFF;
       break;
      }
    case OFF:
      {
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(PUSHD)) {
            digitalWrite(RELAY_PIN, HIGH);
            stateMachine = AUTO;
          }
        }
        break;
      }
    case AUTO:
      {
        isAutoMode = TRUE;
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(PUSHD)) {
            digitalWrite(RELAY_PIN, LOW);
            stateMachine = OFF;
          }
          else if (Encoder.readStatus(PUSHP)) {
            isAutoMode = FALSE;
            stateMachine = MANUAL;
          }
        }
        break;
      }
    case MANUAL:
      {
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(PUSHD)) {
            digitalWrite(RELAY_PIN, LOW);
            stateMachine = OFF;
          }
          else if (Encoder.readStatus(PUSHP)) {
            Encoder.writeGP1(0);
          Encoder.writeGP2(0);
          Encoder.writeGP3(0);
            stateMachine = NIGHT;
          }
          else if (Encoder.readStatus(RINC) || Encoder.readStatus(RDEC))  {
            rotaryPosition = Encoder.readCounterInt();
            Threewire_Control(rotaryPosition);
            DAC_Control(rotaryPosition);
          }
        }
        break;
      }
    case NIGHT:
      {
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(PUSHD)) {
            digitalWrite(RELAY_PIN, LOW);
            stateMachine = OFF;
          }
          else if (Encoder.readStatus(PUSHP)) {
            stateMachine = AUTO;
          }
          else if (Encoder.readStatus(RINC) || Encoder.readStatus(RDEC))  {
            rotaryPosition = Encoder.readCounterInt();
            Threewire_Control(rotaryPosition);
            DAC_Control(rotaryPosition);
          }
        }
        break;
      }
  }
}

void AutoHumididyManager()
{
  bool validdate;
  unsigned int requestedSpeed;
  const unsigned int gain = MAX_SPEED/(DEF_HUMIDITY - MIN_HUMIDITY);
  
  validdate = RefreshDateTime();
  ReadBME280();
  setHumidity = DEF_HUMIDITY;
  if (setHumidity < actualHumidity)
  {
    Threewire_Control(0);
      DAC_Control(0);
  }
  else
  {
    
    requestedSpeed = (setHumidity - actualHumidity)*gain;
    if (requestedSpeed > MAX_SPEED)
    {
      requestedSpeed = MAX_SPEED;
    }
    Threewire_Control(requestedSpeed);
      DAC_Control(requestedSpeed);
  }
  if (validdate)
  {
    if (dateTime.hour < NIGHT_BEFORE || dateTime.hour > NIGHT_AFTER)
    {
      Encoder.writeGP1(0);
        Encoder.writeGP2(0);
        Encoder.writeGP3(0);
    }
  }
}

bool RefreshDateTime()
{
  dateTime = NTPhu.getNTPtime(1.0, 1);
  if (dateTime.year > 2035)
  {
    return 0;
  }
  return (dateTime.valid);
}

void setup() {
  Serial.begin(115200);
  //PCNT_config();
  PWM_config();
  pinMode(BUT_PIN, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, DAC_MAX_VALUE);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);
  delay(100);
  bme.begin(BME280_ADDRESS_ALTERNATE);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,   // mode
                  Adafruit_BME280::SAMPLING_X1,   // temperature
                  Adafruit_BME280::SAMPLING_X1,   // pressure
                  Adafruit_BME280::SAMPLING_X16,  // humidity
                  Adafruit_BME280::FILTER_X16);   // filtering
  delay(100);
  //Encoder.reset();
  Encoder.begin(INT_DATA | WRAP_DISABLE | DIRE_RIGHT | IPUP_DISABLE | RMOD_X1 | STD_ENCODER);
  delay(100);
  Encoder.writeGP1conf(GP_PWM | GP_PULL_DI | GP_INT_DI);  // Configure the GP1 pin in PWM mode
  Encoder.writeGP2conf(GP_PWM | GP_PULL_DI | GP_INT_DI);  // Configure the GP2 pin in PWM mode
  Encoder.writeGP3conf(GP_PWM | GP_PULL_DI | GP_INT_DI);  // Configure the GP3 pin in PWM mode
  Encoder.writeInterruptConfig(RDEC | RINC | PUSHD | PUSHP); /* Enable required interrupts */
  Encoder.writeAntibouncingPeriod(20);  /* Set an anti-bouncing of 200ms */
  Encoder.writeDoublePushPeriod(50);  /*Set a period for the double push of 500ms*/
  Encoder.writeGP1(255);
  Encoder.writeGP2(0);
  Encoder.writeGP3(127);
  attachInterrupt(INTERRUPT_PIN, HandleInterrupt, FALLING);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin (ssid, password);
  // Wait for connection
  unsigned long wifitimeout = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - wifitimeout > TIMEOUT)
    {
      failSafe = TRUE;
      break;
    }
  }
}

void loop() {
  static unsigned long timer;

  if (isAutoMode)
  {
    if (millis() - timer > AUTO_TIMER)
    {
    AutoHumididyManager();
    timer = millis();
    }
  }
  if (rotaryEvent)
  {
    RotaryCheck();
  }
}

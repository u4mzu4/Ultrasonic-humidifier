/*
  ESP32-EVB development board (https://www.olimex.com/Products/IoT/ESP32/ESP32-EVB/open-source-hardware)
  BME280 sensor
  LM2596 programmable power supply
  Sunon MF50152VX-1L01C-S99 blower
  I2C rotary (https://github.com/Fattoresaimon/I2CEncoderV2.1)
  Blynk service
*/


//Includes
#include <driver/pcnt.h>
#include <credentials.h>
#include <Adafruit_BME280.h>
#include <BlynkSimpleEsp32_SSL.h>
#include <i2cEncoderLibV2.h>
#include <NTPtimeESP.h>

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
#define WATERLEVEL_PIN  14
#define INTERRUPT_PIN   17
#define DAC_PIN         25
#define RELAY_PIN       32
#define PCNT_INPUT_PIN  36

#define FALSE           false
#define TRUE            true
#define PWM_FREQ        25000   //25kHz
#define PWM_CHANNEL     0
#define PWM_RESOLUTION  7       //7 bit
#define PWM_MAX_VALUE   127     //2^PWM_RESOLUTION - 1
#define PCNT_FILT_VAL   1023
#define PCNT_L_LIM_VAL  0
#define PCNT_H_LIM_VAL  14400
#define DAC_DEFAULT     96
#define DAC_STEP        38      //6000 / (255 - DAC_DEFAULT)
#define DAC_MAX_VALUE   255     //2^8 - 1
#define RPM_STEP        47      //6000 / PWM_MAX_VALUE
#define MAX_SPEED       5969    //PWM_MAX_VALUE * RPM_STEP
#define SETTLING_TIME   3000    //3 sec
#define DEF_HUMIDITY    60      //DEFAULT AUTO SETTINGS
#define MIN_HUMIDITY    19      //LOWEST MEASURED HUMIDITY EVER
#define AUTO_TIMER      60000   //1 min
#define SPEED_TIMER     1000     //1 sec
#define NIGHT_AFTER     21      //NIGHT START
#define NIGHT_BEFORE    7       //NIGHT END
#define ENCODER_ADDRESS 0x02
#define TIMEOUT         5000    //5 sec
#define YEAR_MIN        2019
#define YEAR_MAX        2035
#define I2C_CLOCK       400000  //400 kHz
#define SERIAL_SPEED    115200
#define BME_TEMP_MIN    1.0     //Celsius
#define BME_TEMP_MAX    100.0   //Celsius
#define BME_PRESS_MIN   870.0   //hPa
#define BME_PRESS_MAX   1086.0  //hPa
#define BME_HUM_MAX     99      //%
#define GP_LED_ON       255
#define GP_LED_OFF      0
#define ENCODER_STEP  300
#define NTPSERVER       "hu.pool.ntp.org"

//Global variables
bool failSafe = FALSE;
unsigned int measRPM;
MAIN_SM stateMachine = INIT;
strDateTime dateTime;

//Init services
Adafruit_BME280 bme;
i2cEncoderLibV2 Encoder(ENCODER_ADDRESS);
NTPtime NTPhu(NTPSERVER);   // Choose server pool as required
//BlynkTimer timer;
WidgetTerminal terminal(V1);

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

void Encoder_config()
{
  Encoder.reset();
  Encoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE | i2cEncoderLibV2::DIRE_RIGHT | i2cEncoderLibV2::IPUP_DISABLE | i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  delay(100);
  Encoder.writeGP1conf(i2cEncoderLibV2::GP_PWM | i2cEncoderLibV2::GP_PULL_DI | i2cEncoderLibV2::GP_INT_DI);  // Configure the GP1 pin in PWM mode
  Encoder.writeGP2conf(i2cEncoderLibV2::GP_PWM | i2cEncoderLibV2::GP_PULL_DI | i2cEncoderLibV2::GP_INT_DI);  // Configure the GP2 pin in PWM mode
  Encoder.writeGP3conf(i2cEncoderLibV2::GP_PWM | i2cEncoderLibV2::GP_PULL_DI | i2cEncoderLibV2::GP_INT_DI);  // Configure the GP3 pin in PWM mode
  Encoder.writeInterruptConfig(i2cEncoderLibV2::RDEC | i2cEncoderLibV2::RINC | i2cEncoderLibV2::PUSHD | i2cEncoderLibV2::PUSHP); /* Enable required interrupts */
  Encoder.writeAntibouncingPeriod(20);  /* Set an anti-bouncing of 200ms */
  Encoder.writeDoublePushPeriod(50);  /*Set a period for the double push of 500ms*/
  Encoder.writeGP1(GP_LED_OFF);
  Encoder.writeGP2(GP_LED_OFF);
  Encoder.writeGP3(GP_LED_OFF);
}

void WiFi_Config()
{
  unsigned long wifitimeout = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin (ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - wifitimeout > TIMEOUT)
    {
      failSafe = TRUE;
      break;
    }
  }
}

void PWM_Control(unsigned int reqSpeed)
{
  unsigned int expReqSpeed;
  static unsigned int prevSpeed;
  static unsigned int tunedSpeed;
  static unsigned long settlingStarted;

  expReqSpeed = Lin2Exp(double(reqSpeed));
  if (abs(expReqSpeed - measRPM) < 2 * RPM_STEP) //speed OK
  {
    return;
  }
  if (prevSpeed != reqSpeed)
  {
    Power_Control(reqSpeed);
    prevSpeed = reqSpeed;
    settlingStarted = millis();
  }
  if (millis() - settlingStarted > SETTLING_TIME) //waiting for settling
  {
    if (measRPM < expReqSpeed) //fine tuning1 needed
    {
      tunedSpeed += RPM_STEP;
      if ((tunedSpeed - reqSpeed) > ENCODER_STEP)
      {
        Serial.println("Speed out of tolerance!");
        return;
      }
      Power_Control(tunedSpeed);
      settlingStarted = millis();
      return;
    }
    if (measRPM > expReqSpeed) //fine tuning1 needed
    {
      tunedSpeed -= RPM_STEP;
      if ((reqSpeed - tunedSpeed) > ENCODER_STEP)
      {
        Serial.println("Speed out of tolerance!");
        return;
      }
      Power_Control(tunedSpeed);
      settlingStarted = millis();
      return;
    }
  }
}

int ReadBME280()
{
  float bmeTemperature;
  float bmeHumidity;
  float bmePressure;
  static float lastvalidTemperature;
  static float lastvalidHumidity;
  static float lastvalidPressure;
  static int bmeErrorcounter = 0;

  bme.takeForcedMeasurement();
  bmeTemperature = bme.readTemperature();
  bmeHumidity = bme.readHumidity();
  bmePressure = bme.readPressure() / 100.0F; //Pa to hPa conversion

  if (bmeTemperature < BME_TEMP_MIN || bmeTemperature > BME_TEMP_MAX) {
    bmeTemperature = lastvalidTemperature;
  }
  else {
    lastvalidTemperature = bmeTemperature;
  }
  if (bmePressure < BME_PRESS_MIN || bmePressure > BME_PRESS_MAX) {
    bmePressure = lastvalidPressure;
  }
  else {
    lastvalidPressure = bmePressure;
  }
  if (bmeHumidity > BME_HUM_MAX) {
    bmeHumidity = lastvalidHumidity;
    bmeErrorcounter++;
  }
  else {
    lastvalidHumidity = bmeHumidity;
    bmeErrorcounter = 0;
  }
  //ErrorManager(BME280_ERROR, bmeErrorcounter, 5);n
  //Blynk.virtualWrite(V2, bmeTemperature);
  //Blynk.virtualWrite(V3, actualHumidity);
  //Blynk.virtualWrite(V4, actualPressure);

  return int(bmeHumidity);
}

void Power_Control(unsigned int reqSpeed)
{
  byte dutyCycle;
  byte dacValue;
  byte ledValue;
  static unsigned int prevSpeed;
  static bool isAutoNight = FALSE;

  if (reqSpeed == 0)
  {
    ledcWrite(PWM_CHANNEL, PWM_MAX_VALUE);
    dacWrite(DAC_PIN, DAC_MAX_VALUE);
    Encoder.writeGP2(GP_LED_OFF);
    return;
  }

  reqSpeed = Lin2Exp(double(reqSpeed));
  if (reqSpeed != prevSpeed) //new PWM needed
  {
    if (AUTO == stateMachine)
    {
      if (RefreshDateTime()) //date & time is valid
      {
        if (dateTime.hour < NIGHT_BEFORE || dateTime.hour >= NIGHT_AFTER)
        {
          isAutoNight = TRUE;
          reqSpeed = 3 * reqSpeed / 4; //75%
        }
        else
        {
          isAutoNight = FALSE;
        }
      }
    }
    dutyCycle = PWM_MAX_VALUE - (reqSpeed / RPM_STEP);
    dacValue = DAC_MAX_VALUE - (reqSpeed / DAC_STEP);
    ledValue = reqSpeed / RPM_STEP;

    Serial.print("PMW duty:");
    Serial.println(PWM_MAX_VALUE - dutyCycle);
    Serial.print("dacValue:");
    Serial.println(dacValue);
    ledcWrite(PWM_CHANNEL, dutyCycle);
    dacWrite(DAC_PIN, dacValue);
    if (isAutoNight || (NIGHT == stateMachine))
    {
      ledValue = GP_LED_OFF;
    }
    Serial.print("ledValue:");
    Serial.println(ledValue);
    Encoder.writeGP2(ledValue);
    prevSpeed = reqSpeed;
  }
}

void RotaryCheck()
{
  unsigned int rotaryPosition;

  switch (stateMachine)
  {
    case INIT:
      {
        Encoder.writeMax((int32_t)MAX_SPEED);  /* Set the maximum threshold */
        Encoder.writeMin((int32_t)0);      /* Set the minimum threshold */
        Encoder.writeStep((int32_t)ENCODER_STEP);
        Encoder.writeCounter((int32_t)0);
        stateMachine = OFF;
        break;
      }
    case OFF:
      {
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(i2cEncoderLibV2::PUSHD)) {
            digitalWrite(RELAY_PIN, HIGH);
            Encoder.writeGP3(GP_LED_OFF);
            AutoHumididyManager(TRUE);
            stateMachine = AUTO;
          }
        }
        break;
      }
    case AUTO:
      {
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(i2cEncoderLibV2::PUSHD)) {
            Power_Control(0);
            digitalWrite(RELAY_PIN, LOW);
            Encoder.writeGP1(GP_LED_OFF);
            Encoder.writeGP3(GP_LED_OFF);
            stateMachine = OFF;
          }
          else if (Encoder.readStatus(i2cEncoderLibV2::PUSHP)) {
            Encoder.writeGP1(GP_LED_ON);
            rotaryPosition = Encoder.readCounterInt();
            Power_Control(rotaryPosition);
            stateMachine = MANUAL;
          }
        }
        break;
      }
    case MANUAL:
      {
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(i2cEncoderLibV2::PUSHD)) {
            Power_Control(0);
            digitalWrite(RELAY_PIN, LOW);
            Encoder.writeGP1(GP_LED_OFF);
            Encoder.writeGP3(GP_LED_OFF);
            stateMachine = OFF;
          }
          else if (Encoder.readStatus(i2cEncoderLibV2::PUSHP)) {
            Encoder.writeGP1(GP_LED_OFF);
            Encoder.writeGP2(GP_LED_OFF);
            stateMachine = NIGHT;
          }
          else if (Encoder.readStatus(i2cEncoderLibV2::RINC) || Encoder.readStatus(i2cEncoderLibV2::RDEC))  {
            rotaryPosition = Encoder.readCounterInt();
            Power_Control(rotaryPosition);
          }
        }
        break;
      }
    case NIGHT:
      {
        if (Encoder.updateStatus()) {
          if (Encoder.readStatus(i2cEncoderLibV2::PUSHD)) {
            Power_Control(0);
            digitalWrite(RELAY_PIN, LOW);
            Encoder.writeGP1(GP_LED_OFF);
            Encoder.writeGP3(GP_LED_OFF);
            stateMachine = OFF;
          }
          else if (Encoder.readStatus(i2cEncoderLibV2::PUSHP)) {
            AutoHumididyManager(TRUE);
            stateMachine = AUTO;
          }
          else if (Encoder.readStatus(i2cEncoderLibV2::RINC) || Encoder.readStatus(i2cEncoderLibV2::RDEC))  {
            rotaryPosition = Encoder.readCounterInt();
            Power_Control(rotaryPosition);
          }
        }
        break;
      }
  }
}

void AutoHumididyManager(bool forced)
{
  unsigned int actualHumidity;
  unsigned int requestedSpeed;
  const unsigned int gain = MAX_SPEED / (DEF_HUMIDITY - MIN_HUMIDITY);
  static unsigned long timer;

  if (millis() - timer > AUTO_TIMER)
  {
    actualHumidity = ReadBME280();
    Serial.println("Humidity:");
    Serial.println(actualHumidity);
    if (DEF_HUMIDITY < actualHumidity)
    {
      Power_Control(0);
    }
    else
    {
      requestedSpeed = (DEF_HUMIDITY - actualHumidity) * gain;
      if (requestedSpeed > MAX_SPEED)
      {
        requestedSpeed = MAX_SPEED;
      }
      Power_Control(requestedSpeed);
    }
    timer = millis();
  }
}

bool RefreshDateTime()
{
  if (failSafe)
  {
    return FALSE;
  }
  dateTime = NTPhu.getNTPtime(1.0, 1);
  if ((dateTime.year < YEAR_MIN) || (dateTime.year > YEAR_MAX))
  {
    return FALSE;
  }
  return dateTime.valid;
}

void TankEmtyProcess()
{
  Power_Control(0);
  digitalWrite(RELAY_PIN, LOW);
  Encoder.writeGP3(GP_LED_ON);
  Encoder.writeGP1(GP_LED_OFF);
  Encoder.writeGP2(GP_LED_OFF);
  stateMachine = OFF;
}

unsigned int Lin2Exp (double linearinput)
{
  double expoutput = linearinput * exp(1 + (-1 * linearinput / double(MAX_SPEED)));
  return round(expoutput);
}

void Calculate_Speed()
{
  short counter;
  static unsigned long stimer;

  if (millis() - stimer > SPEED_TIMER)
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &counter);
    measRPM = 30 * counter; //60 * counter / 2
    pcnt_counter_clear(PCNT_UNIT_0);
    stimer = millis();
  }
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  PCNT_config();
  PWM_config();
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(WATERLEVEL_PIN, INPUT_PULLUP);
  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, DAC_MAX_VALUE);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Wire.begin(SDA, SCL);
  Wire.setClock(I2C_CLOCK);
  delay(100);
  bme.begin(BME280_ADDRESS_ALTERNATE);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,   // mode
                  Adafruit_BME280::SAMPLING_X1,   // temperature
                  Adafruit_BME280::SAMPLING_X1,   // pressure
                  Adafruit_BME280::SAMPLING_X16,  // humidity
                  Adafruit_BME280::FILTER_X16);   // filtering
  delay(100);
  Encoder_config();
  WiFi_Config();
  RotaryCheck();
}

void loop() {
  bool tankIsEmpty;
  bool rotaryEvent;

  tankIsEmpty = !digitalRead(WATERLEVEL_PIN);
  rotaryEvent = !digitalRead(INTERRUPT_PIN);

  if (tankIsEmpty)
  {
    TankEmtyProcess();
  }
  if (rotaryEvent)
  {
    RotaryCheck();
  }
  /* if (OFF != stateMachine)
    {
      Calculate_Speed();
    }
  */
  if (AUTO == stateMachine)
  {
    AutoHumididyManager(FALSE);
  }
}

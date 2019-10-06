/*
MSENS - pinout
---------------
PIN - color     - name
 35 - žlutý     - TL_1
 37 - zelený    - LED1
 39 - oranžový  -LED2
 41 - modrý     - TL_2 
 6  - bílý      - WS2812 TBD

Zatop LED: 
ON - 


TODO:
- funkce na zobrazovani hodnot (premazavani 0)
- PID screen - zobrazeni sipky smeru otaceni
              - Overflow pøi -30000 -> -30
- Vyhasina - s -> minuty
- Zjistit rychlost smyèky
- nevyskoèí to z "vychlazení"


*/

// Include the libraries that we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <LcdProgressBar.h>
#include <SerialCommand.h>
#include <AutoPID.h>
#include <JC_Button.h>
#include <my-utils.h>

SerialCommand SCmd; // The SerialCommand object

#define BACKLIGHT_TIME_TO_OFF 600000 // in ms
LiquidCrystal_I2C lcd(0x3f, 20, 4);
LcdProgressBar lpg(&lcd, 3, 20);

// PINS definition
#define ONE_WIRE_BUS 8
#define BUT_1_P 35
#define BL1 37 //Button LED 1
#define BUT_2_P 41
#define BL2 39
#define BUT_N_P 22
#define BUT_P_P 23

#define THERMOVCC 10
#define THERMOGND 9
#define THERMODO_P 13
#define THERMOCS_P 12
#define THERMOCLK_P 11

#define VALVE_OPEN_P 26
#define VALVE_CLOSE_P 27
#define PUMP_P 28
#define PUMP_TUV_P 29
#define BOILER_FAN_P 30
#define EXT_FAILSAFE_P 42
// 31, 32, 33 UNUSED RELAYS

MAX6675 thermocouple(THERMOCLK_P, THERMOCS_P, THERMODO_P);

//Buttons
Button BT1(BUT_1_P);
Button BT2(BUT_2_P);
Button BTN(BUT_N_P);
Button BTP(BUT_P_P);

// Dallas
#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress boiler_in = {0x28, 0xFF, 0x48, 0x45, 0x40, 0x17, 0x04, 0x2C};    // 0
DeviceAddress boiler_out = {0x28, 0xFF, 0x64, 0x33, 0x73, 0x16, 0x05, 0xf4};   // 1
DeviceAddress house_return = {0x28, 0xFF, 0x54, 0x01, 0x40, 0x17, 0x03, 0x1C}; // 2
DeviceAddress house_in = {0x28, 0xFF, 0x61, 0x04, 0x40, 0x17, 0x03, 0xCB};     // 3
DeviceAddress TUV_out = {0x28, 0xFF, 0xA5, 0x21, 0x40, 0x17, 0x04, 0x84};

double t_boiler_in,
    t_boiler_out, t_house_return, t_house_in, outputVal_in, outputVal_out, t_TUV_out, t_komin = 0.0;

unsigned long backlight_lastT = 0;
unsigned long last_mode_start = 0;
unsigned long stop_timeout_T = 0;

// konstanty
#define VYHASNUTO 40.0 // od kdy se má zapnout normální režim
#define TEMPHYST 3.0
#define FAILURETEMPERATURE 95.0   //STOP PID, just open valve and
#define VENTILATORMAXTEPLOTA 75.0 //TEPLOTA VODY KDY SE NIKDY NESEPNE (teplota na výstupu)
#define FAN_VYHASNUTO_TEMP 150.0  // Teplota na ventilátoru od kdy se bere, že kotel hoøí

#define TUV_MAX_TRY 3             //maximum try to heat TUV
#define TUV_MIN_DIFF_ON 2         //minimum difference between t_boiler_in - t_boiler_out to turn on TUV
#define TUV_MIN_DIFF_OFF 1.0      //diff between t_boiler_in - t_boiler_out, when stop TUV PUMP
const double MAX_TUV_TEMP = 75.0; // kdy už se TUV bere jako natopene

#define FAILSAFE_MODE 5
#define BOILER_IN_T 70
#define BOILER_IN_TUV_T 75

double set_boiler_in = BOILER_IN_T;
double set_boiler_out = 90.0;

bool blik_p, slow_p, fast_p = 0; //for blinking
int disp_mode, TUV_mode, mode = 0;
bool TUV_reguest = 1;
unsigned long fan_force_on = 0;
int TUV_try = 0;

//pid settings and gains
const double OUTPUT_MIN = -30000.0;
const double OUTPUT_MAX = 30000.0;
double KP_in = 1500.0;
double KI_in = 1.0;
double KD_in = 5.0;

double KP_out = 500.0;
double KI_out = 5.0;
double KD_out = 0.0;

AutoPID myPID_in(&t_boiler_in, &set_boiler_in, &outputVal_in, OUTPUT_MIN, OUTPUT_MAX, KP_in, KI_in, KD_in);
AutoPID myPID_out(&t_boiler_out, &set_boiler_out, &outputVal_out, OUTPUT_MIN, OUTPUT_MAX, KP_out, KI_out, KD_out);

//
bool temperature_read(void);
void print_temperatures(void);
void valve_control(void);
void disp(void);
void reset_backlight(void);
void valve(int x);
bool pumpControl(void);
bool fanControl(void);
bool ledControl(void);
bool TUV(void);

void setup(void)
{

  // start serial port
  Serial.begin(115200);
  Serial.print("Kotel:  ");
  Serial.println(__DATE__);

  // Display welcome screen
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Kotel init...");
  lcd.setCursor(0, 2);
  lcd.print("Build date:");
  lcd.setCursor(9, 3);
  lcd.print(__DATE__);

  //Pins MODE
  pinMode(BL1, OUTPUT);
  pinMode(BL2, OUTPUT);
  pinMode(VALVE_CLOSE_P, OUTPUT);
  pinMode(VALVE_OPEN_P, OUTPUT);
  pinMode(PUMP_P, OUTPUT);
  pinMode(BOILER_FAN_P, OUTPUT);
  pinMode(PUMP_TUV_P, OUTPUT);
  pinMode(EXT_FAILSAFE_P, INPUT_PULLUP);

  digitalWrite(PUMP_TUV_P, HIGH);
  digitalWrite(BOILER_FAN_P, HIGH);
  pinMode(THERMOGND, OUTPUT);
  pinMode(THERMOVCC, OUTPUT);
  digitalWrite(THERMOGND, LOW);
  digitalWrite(THERMOVCC, HIGH);

  // if program stop, these two relays are on
  digitalWrite(PUMP_P, LOW);
  valve(1);

  //Buttons handlers
  BT1.begin();
  BT2.begin();
  BTN.begin();
  BTP.begin();

  //--------------------- comand register
  SCmd.addCommand("t", print_temperatures);
  SCmd.addCommand("p", print_pid);
  SCmd.addCommand("r", pid_reset);
  SCmd.addCommand("set", set_pid);
  SCmd.addDefaultHandler(unrecognized);

  //--------------------- comand register

  sensors.begin(); // Dalas start
  sensors.setResolution(boiler_in, TEMPERATURE_PRECISION);
  sensors.setResolution(boiler_out, TEMPERATURE_PRECISION);
  sensors.setResolution(house_return, TEMPERATURE_PRECISION);
  sensors.setResolution(house_in, TEMPERATURE_PRECISION);
  sensors.setWaitForConversion(true);
  sensors.requestTemperatures();
  temperature_read();
  sensors.setWaitForConversion(false);

  // Get dalas count
  Serial.print("Found  ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" Dalas devices.");

  while (!temperature_read())
  {
  } //wait until temp sensor updated
  print_temperatures();

  myPID_in.setBangBang(6);
  myPID_in.setTimeStep(1000);
  myPID_out.setBangBang((FAILURETEMPERATURE - set_boiler_out)-1);
  myPID_out.setTimeStep(1000);
  Serial.println("...start...");
  delay(1000);
  valve(0);
  digitalWrite(PUMP_P, HIGH);
  lcd.clear();

  if ((t_boiler_out > VYHASNUTO - 2 * TEMPHYST) || t_komin > FAN_VYHASNUTO_TEMP - 2 * TEMPHYST) // worse case senario, reset during startup
  {
    mode = 2;
  }
}

void valve(int x = 0)
{
  // 0  - stop
  // 1  - OPEN
  // -1 - CLOSE
  if (x == 1)
  {
    digitalWrite(VALVE_OPEN_P, LOW);
    digitalWrite(VALVE_CLOSE_P, HIGH);
  }
  else if (x == -1)
  {
    digitalWrite(VALVE_OPEN_P, HIGH);
    digitalWrite(VALVE_CLOSE_P, LOW);
  }
  else
  {
    digitalWrite(VALVE_OPEN_P, HIGH);
    digitalWrite(VALVE_CLOSE_P, HIGH);
  }
}

#define STOP_TIMEOUT 1200000
void modeChange()
{
  static int last_mode = 0;
  static int prilozeno = 0;

  static float last_temp = 0.0;
  switch (mode)
  {
  case 0: // STOP do nothing
    if (BT1.wasReleased())
    { // pøikaz na zatop
      mode = 1;
      prilozeno = 1;
    }
    if ((t_boiler_out > VYHASNUTO + 4 * TEMPHYST) || t_komin > FAN_VYHASNUTO_TEMP + TEMPHYST)
    { // není nahodou moc teplo?
      mode = 4;
      prilozeno = 0;
      break;
    }
    if (last_mode != mode)
    {
      TUV_try = 0; //reset TUV max try
      TUV_reguest = 1;
      last_mode_start = millis();
      disp_mode = mode;
    }
    last_mode = 0;

    fan_force_on = 0;
    break;

  case 1: // zatop
    if (BT1.wasReleased())
    {
      mode = 0;
    }
    if (t_boiler_out > set_boiler_in + TEMPHYST)
    { //výstup je vìtší neš požadovaný vstup
      prilozeno = 0;
      mode = 2;
    }
    if ((t_boiler_out < VYHASNUTO - TEMPHYST) && prilozeno == 0) // prilozeno to know is is just start to fire
    {
      mode = 4;
    }

    if (last_mode != mode)
    {
      last_mode_start = millis();
      disp_mode = mode;
    }
    last_mode = 1;
    break;

  case 2: // normalni chod

    if (t_boiler_out < BOILER_IN_T)
    {
      mode = 3;
    }
    if (last_mode != mode)
    {
      last_mode_start = millis();
      disp_mode = mode;
    }
    last_mode = 2;

    break;

  case 3:                  //vyhasina, potøebuje pøiložit
    if (BT1.wasReleased()) //prilozeno
    {                      // znovu prilozeno
      mode = 1;
    }
    if (t_boiler_out > BOILER_IN_T + TEMPHYST) //znovu hoøí
    {
      mode = 2;
    }

    if (last_mode != mode) //start
    {
      last_mode_start = millis();
      disp_mode = mode;
      stop_timeout_T = millis();
    }
    if ((millis() > stop_timeout_T + STOP_TIMEOUT) || (t_boiler_out < VYHASNUTO - TEMPHYST)) // ukonèit
      mode = 4;

    last_mode = 3;

    break;

  case 4: // vyèerpat teplou vodu
    if (BT1.wasReleased())
    { // znovu prilozeno
      mode = 1;
    }
    if ((t_boiler_out < VYHASNUTO) && (t_komin < FAN_VYHASNUTO_TEMP))
    {
      mode = 0;
    }
    if (last_mode != mode)
    {
      last_mode_start = millis();
      disp_mode = mode;
    }
    last_mode = 4;

    break;
  case FAILSAFE_MODE: //failsafe mode, only from this place, failsafe can be CLOSE
    if (t_boiler_out < FAILURETEMPERATURE - TEMPHYST && t_boiler_in < FAILURETEMPERATURE - (2 * TEMPHYST))
    {
      if ((t_boiler_in > 0.0) && (t_boiler_out > 0.0)) //check if sensors are connected, DEVICE_DISCONNECTED_C = -127
      {
        mode = 2;
      }
    }

    if (last_mode != mode)
    {
      last_mode_start = millis();
      disp_mode = mode;
    }
    last_mode = FAILSAFE_MODE;
    break;
  }
}

bool failsafeCheck(void)
{
  if (!digitalRead(EXT_FAILSAFE_P))
  {
    mode = FAILSAFE_MODE;
    return;
  }
  if (t_boiler_out == DEVICE_DISCONNECTED_C)
  {
    mode = FAILSAFE_MODE;
    return;
  }

  if (t_boiler_in == DEVICE_DISCONNECTED_C)
  {
    mode = FAILSAFE_MODE;
    return;
  }

  if (t_boiler_out > FAILURETEMPERATURE)
  {
    mode = FAILSAFE_MODE;
    return;
  }

  if (t_boiler_in > FAILURETEMPERATURE)
  {
    mode = FAILSAFE_MODE;
    return;
  }
}

bool pumpControl(void)
{

  switch (mode)
  {
  case 0:
    digitalWrite(PUMP_P, HIGH);
    break;
  default:
    digitalWrite(PUMP_P, LOW);
    break;
  }

  return !digitalRead(PUMP_P);
}

#define FAN_TIME_PRIKLADANI 60000
bool fanControl(void)
{

  if (BT1.isPressed())
  {
    fan_force_on = millis() + FAN_TIME_PRIKLADANI;
  }

  if ((fan_force_on > millis()) && (t_boiler_out < FAILURETEMPERATURE - 2))
  {
    digitalWrite(BOILER_FAN_P, LOW);
  }
  else
  {

    switch (mode)
    {
    case 1: //zatop
      digitalWrite(BOILER_FAN_P, LOW);
      break;
    case 2:                                    // topi
      if (t_boiler_out < VENTILATORMAXTEPLOTA) //vysoká teplota na výstupu, nespínat ventilátor
      {
        digitalWrite(BOILER_FAN_P, LOW);
      }
      else
      {
        digitalWrite(BOILER_FAN_P, HIGH);
      }
      break;
    case 3: // vyhasina
      digitalWrite(BOILER_FAN_P, LOW);
      break;
    case 4: // vychlazeni
      digitalWrite(BOILER_FAN_P, LOW);
      break;
    default:
      digitalWrite(BOILER_FAN_P, HIGH);
      break;
    }
  }
  return !digitalRead(BOILER_FAN_P);
}

bool ledControl(void)
{

  switch (mode)
  {
  case 0:
    digitalWrite(BL1, blik_p ? HIGH : LOW);
    break;
  case 1: //zátop
    digitalWrite(BL1, slow_p ? LOW : HIGH);
    break;
  case 2: //topi normal
    digitalWrite(BL1, HIGH);
    break;
  case 3: // potøebuje prilozit
    digitalWrite(BL1, fast_p ? LOW : HIGH);
    break;
  case 4: // Vyhasíná
    digitalWrite(BL1, fast_p ? LOW : HIGH);
    break;
  default:
    digitalWrite(BL1, HIGH);
    break;
  }
  switch (TUV_mode)
  {
    /*
TUV_mode:
0 stop
1 wait
2 topit
3 natopeno
*/
  case 0:
    digitalWrite(BL2, LOW);
    break;
  case 1:
    digitalWrite(BL2, slow_p ? LOW : HIGH);
    break;
  case 2:
    digitalWrite(BL2, fast_p ? LOW : HIGH);
    break;
  case 3:
    digitalWrite(BL2, HIGH);
    break;
  default:
    digitalWrite(BL2, HIGH);
    break;
  }
}

void unrecognized()
{
  Serial.println("What?");
  Serial.println("> t - print_temperatures");
  Serial.println("> p - print_pid");
  Serial.println("> r - pid_reset");
  Serial.println("> set PID OUT, PID IN(1-7) 100 , /1000");
}

void pid_reset()
{
  Serial.println("PID reset");
  myPID_in.reset();
  myPID_out.reset();
}

void print_temperatures(void)
{
  Serial.println();
  Serial.println("Temperature print:");
  Serial.print("> 0 boiler_in: ");
  Serial.println(t_boiler_in, 1);
  Serial.print("> 1 boiler_out: ");
  Serial.println(t_boiler_out, 1);
  Serial.print("> 2 house_return: ");
  Serial.println(t_house_return, 1);
  Serial.print("> 3 house_in: ");
  Serial.println(t_house_in, 1);
  Serial.print("> 4 komin: ");
  Serial.println(t_komin, 1);
  Serial.print("> 5 TUV: ");
  Serial.println(t_TUV_out, 1);
  Serial.println();
}

void print_pid(void)
{
  Serial.println();
  Serial.println("PID print:");
  Serial.print("> outputVal_in: ");
  Serial.println(outputVal_in);
  Serial.print("> outputVal_out: ");
  Serial.println(outputVal_out);
  Serial.println();
}

void set_pid()
{

  int aNumber;
  int mode;
  char *arg;

  //set kpo 10
  arg = SCmd.next();
  if (arg != NULL)
  {
    mode = atol(arg);
    arg = SCmd.next();
    if (arg != NULL)
    {
      aNumber = atol(arg);
    }
    else
    {
      Serial.println("No second argument");
      return;
    }
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
  aNumber = aNumber / 1000;
  switch (mode)
  {
  case 1:
    Serial.println("KP - out set");
    KP_out = aNumber;
    break;
  case 2:
    KI_out = aNumber;
    Serial.println("KI - out set");
    break;
  case 3:
    KD_out = aNumber;
    Serial.println("KD - out set");
    break;
  case 4:
    KP_in = aNumber;
    Serial.println("KP - in set");
    break;
  case 5:
    KI_in = aNumber;
    Serial.println("KI - in set");
    break;
  case 6:
    KD_in = aNumber;
    Serial.println("KD - in set");
    break;
  }
  Serial.println("");
  Serial.println("123 - PID OUT:");
  Serial.println(KP_out, 5);
  Serial.println(KI_out, 5);
  Serial.println(KD_out, 5);
  Serial.println("");
  Serial.println("456 - PID IN:");
  Serial.println(KP_in, 5);
  Serial.println(KI_in, 5);
  Serial.println(KD_in, 5);

  myPID_in.setGains(KP_in, KI_in, KD_in);
  myPID_out.setGains(KP_out, KI_out, KD_out);
}

bool temperature_read(void)
{
  //read all temperatures and store it in global variable
  const int conversion_delay = 2000;
  static unsigned long lastT = 0;

  if ((millis() > lastT + conversion_delay))
  {
    t_boiler_in = sensors.getTempC(boiler_in);
    t_boiler_out = sensors.getTempC(boiler_out);
    t_house_in = sensors.getTempC(house_in);
    t_house_return = sensors.getTempC(house_return);
    t_TUV_out = sensors.getTempC(TUV_out);

    sensors.requestTemperatures();
    lastT = millis();

    t_komin = thermocouple.readCelsius(); //read thermocouple with dalas delay
    return true;
  }
  return false;
}

bool TUV(void)
{
  if (BT2.wasReleased())
  {
    TUV_reguest = !TUV_reguest;
  }
  if (TUV_reguest == 0)
  {
    TUV_mode = 0;
  }
  if (TUV_mode == 2)
  {

    // v momente kdy se sepne topeni boileru, tak snizit tepluto vratky
    // jinak když se vypne topeni do boileru vznikne prekmit

    set_boiler_in = BOILER_IN_TUV_T;
  }
  else
  {
    set_boiler_in = BOILER_IN_T;
  }

  switch (TUV_mode)
  {
  case 0:
    if (TUV_reguest && (mode != 0)) //get ready TUV
    {
      TUV_mode = 1;
      TUV_try = 0;
    }
    break;
  case 1: //wait
    if (((t_boiler_out - t_boiler_in) > TUV_MIN_DIFF_ON) && (t_TUV_out < t_boiler_out + TEMPHYST))
    {
      TUV_mode = 2;
    }
    break;
  case 2:                           //topit
    if ((t_TUV_out > MAX_TUV_TEMP)) //je natopený boiler, hotovo
    {
      TUV_mode = 3;
    }

    if (((t_boiler_out - t_boiler_in) < TUV_MIN_DIFF_OFF) && (t_TUV_out > t_boiler_out))
    {
      TUV_mode = 1;
      TUV_try++;
    }
    if (TUV_try > TUV_MAX_TRY)
    {
      TUV_mode = 0;
      TUV_reguest = 0;
    }

    break;

  case 3: //natopeno
    if (mode == 0)
    {
      TUV_mode = 0;
    }
    break;
  default:
    TUV_mode = 0;
    break;
  }
  /*
TUV_mode:
0 stop
1 wait
2 topit
3 natopeno
*/

  switch (mode)
  {
  case FAILSAFE_MODE: //failsafe, turn on pump
    digitalWrite(PUMP_TUV_P, LOW);
    TUV_mode = 2;
    break;
  case 2: // pouze zde topit do boileru
    switch (TUV_mode)
    {
    case 2:
      digitalWrite(PUMP_TUV_P, LOW);
      break;
    default:
      digitalWrite(PUMP_TUV_P, HIGH);
      break;
    }
    break;
  default:
    digitalWrite(PUMP_TUV_P, HIGH);
    if (TUV_mode == 2) // hlavní mode není ok na topení, tak TUV_mode pøepnout na wait, pokud je požadavek na TUV
    {
      TUV_mode = 1;
    }
    break;
  }

  //kdykoli je STOP, tak ani TUV nemá bezet, reguest ale stále platí
  if (mode == 0)
    TUV_mode = 0;

  return !digitalRead(PUMP_TUV_P);
}

void pulse(void)
{
// periodically blink, use int LEDs blinking
#define SLOW_PERIOD 700
#define FAST_PERIOD 100
#define BLIK_DELAY 6000
  static unsigned long bl_period = BLIK_DELAY;
  static unsigned long lastT_s = 0;
  static unsigned long lastT_f = 0;
  static unsigned long lastT_b = 0;
  unsigned long now = millis();

  if (now > bl_period + lastT_b)
  {

    blik_p = !blik_p;
    lastT_b = now;
    if (blik_p)
    {
      bl_period = FAST_PERIOD;
    }
    else
    {
      bl_period = BLIK_DELAY;
    }
  }

  if (now > SLOW_PERIOD + lastT_s)
  {
    slow_p = !slow_p;
    lastT_s = now;
  }

  if (now > FAST_PERIOD + lastT_f)
  {
    fast_p = !fast_p;
    lastT_f = now;
  }
}

#define PID_DEBUG
#define VALVE_MIN_ON_TIME 2000 //ms
#define VALVE_MAX_T 120000
#define DIRECTION_CHANGE_ADD 100 //When valve change direction, add some time to reduce clearance
void valve_control(void)
{
  unsigned long now = millis();
  unsigned long WindowSize = 30000;
  static unsigned long windowStartTime = 0;
  static int x = 0;
  static int last_D = 0; //last direction
  static long total_on_time = 0;

  double OutputVal = min(outputVal_in, outputVal_out);
  // Less value mean open, to cool input water
  switch (mode)
  {
  case FAILSAFE_MODE: //open valve at failsafe
    valve(1);
    last_D = 0;
    total_on_time = 0;
    pid_reset(); //something wrong with PID, do reset
    windowStartTime = now;
    break;

  case 4: //vychladnuti
    valve(1);
    last_D = 0;
    total_on_time = 0;
    pid_reset();
    windowStartTime = now;
    break;

  default:
    if (now - windowStartTime > WindowSize)
    { //time to shift the Relay Window
#ifdef PID_DEBUG
      Serial.println("---PID_debug---");
      Serial.print("OutputVal: ");
      Serial.println(OutputVal);
      Serial.print("total_on_time: ");
      Serial.println(total_on_time);
#endif

      windowStartTime += WindowSize;
      if ((last_D == -1) && (OutputVal < 0) || (last_D == 1) && (OutputVal > 0))
      {
        // no direction change
        total_on_time += abs(OutputVal); //increment total on time
      }
      else
      {
        // change direction
        if (OutputVal < 0)
        {
          last_D = -1;
          total_on_time = 0;
        }
        else if (OutputVal > 0)
        {
          last_D = 1;
          total_on_time = 0;
        }
        else
        {
          last_D = 0;
          total_on_time = 0;
        }
      }
    }

    if (OutputVal > VALVE_MIN_ON_TIME) //OPEN
    {
      if (x == -1)
      {
        OutputVal += DIRECTION_CHANGE_ADD;
      }
      x = 1;
    }
    else if (OutputVal < -VALVE_MIN_ON_TIME) //CLOSE
    {
      OutputVal = abs(OutputVal);
      if (x == 1)
      {
        OutputVal += DIRECTION_CHANGE_ADD;
      }
      x = -1;
    }
    else // NOTHING
    {
      x = 0;
    }

    if (OutputVal > now - windowStartTime && total_on_time < VALVE_MAX_T && x != 0)
    {
      valve(-x);
    }
    else
    {
      valve(0);
    }
    break;
  }
}

void reset_backlight(void)
{
  backlight_lastT = millis();
  lcd.backlight();
}

void disp_show_time(int m, int h, int x = 14, int y = 0)
{
  lcd.setCursor(x, y);
  if (h >= 10)
  {
    lcd.print(h);
  }
  else
  {
    lcd.print(" ");
    lcd.print(h);
  }
  lcd.print(":");
  if (m >= 10)
  {
    lcd.print(m);
  }
  else
  {
    lcd.print("0");
    lcd.print(m);
  }
}

void disp(void)
{
  static int last_disp_mode = 0;
  unsigned long last_mode_duration_m = ((millis() - last_mode_start) / 60000) % 60; // in minutes
  unsigned long last_mode_duration_h = ((millis() - last_mode_start) / 60000) / 60; // in minutes

  if (BT1.isPressed() || BT2.isPressed() || BTN.isPressed() || BTP.isPressed()) //turn backlight on if any key is pressed
  {
    reset_backlight();
  }

  if (millis() > backlight_lastT + BACKLIGHT_TIME_TO_OFF) //turn backlight off after some time
  {
    lcd.noBacklight();
  }
  if (BTN.wasReleased())
    disp_mode++;
  if (BTP.wasReleased()) //go home
    disp_mode = 0;

  if (last_disp_mode != disp_mode)
  {
    lcd.clear();
    last_disp_mode = disp_mode;
  }

  switch (disp_mode)
  {
  case 0:
    if (mode != disp_mode)
    {
      disp_mode++;
      break;
    }
    lcd.setCursor(0, 0);
    lcd.print("STOP");
    disp_show_time(last_mode_duration_m, last_mode_duration_h);
    lcd.setCursor(0, 1);
    lcd.print("Zmackni ZATOP");

    break;
  case 1:
    if (mode != disp_mode)
    {
      disp_mode++;
      break;
    }
    lcd.setCursor(0, 0);
    lcd.print("Zatop");
    disp_show_time(last_mode_duration_m, last_mode_duration_h);
    lcd.setCursor(0, 1);
    lcd.print("Komin: ");
    lcd.print(t_komin, 0);
    lcd.setCursor(0, 2);
    lcd.print("Vystup kotle:  ");
    lcd.print(t_boiler_out, 1);
    lpg.setMinValue(20);
    lpg.setMaxValue(set_boiler_in);
    lpg.reDraw(t_boiler_in);

    break;
  case 2:
    if (mode != disp_mode)
    {
      disp_mode++;
      break;
    }
    lcd.setCursor(0, 0);
    lcd.print("TOPI");
    disp_show_time(last_mode_duration_m, last_mode_duration_h);
    lcd.setCursor(0, 1);
    lcd.print("Komin: ");
    lcd.print(t_komin, 0);
    lcd.setCursor(0, 2);
    lcd.print("Vystup kotle:  ");
    lcd.print(t_boiler_out, 1);
    lpg.setMinValue(0);
    lpg.setMaxValue(set_boiler_out - set_boiler_in);
    lpg.reDraw(t_boiler_out - t_boiler_in);

    break;
  case 3:
    if (mode != disp_mode)
    {
      disp_mode++;
      break;
    }
    lcd.setCursor(0, 0);
    lcd.print("Vyhasina ");
    disp_show_time(last_mode_duration_m, last_mode_duration_h);
    lcd.setCursor(0, 1);
    lcd.print("Zmackni ZATOP");

    lcd.setCursor(0, 2);
    lcd.print("Konec za: ");
    lcd.print(abs((stop_timeout_T + STOP_TIMEOUT) - millis()) / 1000);
    lpg.setMinValue(VYHASNUTO - TEMPHYST);
    lpg.setMaxValue(set_boiler_in);
    lpg.reDraw(t_boiler_out);

    break;
  case 4:
    if (mode != disp_mode)
    {
      disp_mode++;
      break;
    }
    lcd.setCursor(0, 0);
    lcd.print("Vychlazeni ");
    disp_show_time(last_mode_duration_m, last_mode_duration_h);
    lcd.setCursor(0, 1);
    lcd.print("Zmackni ZATOP");
    lcd.setCursor(0, 2);
    lcd.print("Vystup: ");
    lcd.print(t_boiler_out);
    lpg.setMinValue(VYHASNUTO - (4 * TEMPHYST));
    lpg.setMaxValue(set_boiler_in);
    lpg.reDraw(t_boiler_out);

    break;

  case FAILSAFE_MODE:
    if (mode != disp_mode)
    {
      disp_mode++;
      break;
    }
    lcd.setCursor(0, 0);
    lcd.print("Failsafe ");
    disp_show_time(last_mode_duration_m, last_mode_duration_h);
    lcd.setCursor(0, 1);
    lcd.print("Komin: ");
    lcd.print(t_komin, 0);
    lcd.setCursor(0, 2);
    lcd.print("Vystup kotle:  ");
    lcd.print(t_boiler_out, 1);
    lcd.setCursor(0, 3);
    lcd.print("Vstup kotle:  ");
    lcd.print(t_boiler_in, 1);

    break;

  case 6: // valve
    lcd.setCursor(0, 0);
    lcd.print("pid_IN  ------ ");
    lcd.print(outputVal_in, 0);
    lcd.setCursor(0, 1);
    lcd.print("t_IN : ");
    lcd.print(t_boiler_in, 1);
    lcd.print(" #set: ");
    lcd.print(set_boiler_in, 0);
    lcd.setCursor(0, 2);
    lcd.print("pid_OUT ------ ");
    lcd.print(outputVal_out, 0);
    lcd.setCursor(0, 3);
    lcd.print("t_OUT: ");
    lcd.print(t_boiler_out, 1);
    lcd.print(" #set: ");
    lcd.print(set_boiler_out, 0);

    break;
  case 7:
    lcd.setCursor(0, 0);
    lcd.print("Teploty kotel:");
    lcd.setCursor(0, 1);
    lcd.print("Vystup: ");
    lcd.print(t_boiler_out, 0);
    lcd.print("  Vrat: ");
    lcd.print(t_boiler_in, 0);
    lcd.setCursor(0, 2);
    lcd.print("Komin :  ");
    lcd.print(t_komin);
    lcd.print(" | Dum:");
    lcd.setCursor(0, 3);
    lcd.print("Vstup: ");
    lcd.print(t_house_in, 0);
    lcd.print("  Vrat: ");
    lcd.print(t_house_return, 0);

    break;
  case 8:
    lcd.setCursor(0, 0);
    lcd.print("TUV: ");
    lcd.print(t_TUV_out, 1);
    lcd.setCursor(0, 1);
    lcd.print("TUV pozadovana: ");
    lcd.print(MAX_TUV_TEMP, 1);
    lcd.setCursor(0, 2);
    lcd.print("Try: ");
    lcd.print(TUV_try);
    lcd.print(" Mode:");
    lcd.print(TUV_mode);
    lpg.setMinValue(40);
    lpg.setMaxValue(MAX_TUV_TEMP);
    lpg.reDraw(t_TUV_out);
    break;

  default:
    disp_mode = 0;
    break;
  }
}

//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================

void loop(void)
{

  temperature_read();
  SCmd.readSerial();
  myPID_in.run();
  myPID_out.run();
  pulse();
  disp();
  valve_control();
  modeChange();
  pumpControl();
  ledControl();
  TUV();
  fanControl();
  failsafeCheck();

  //BTN
  BT1.read();
  BT2.read();
  BTN.read();
  BTP.read();
}

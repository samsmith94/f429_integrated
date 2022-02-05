#include <Arduino.h>
#include <pins.h>

#include <Relay.h>
#include <MOSFET.h>

#include <Servo.h>

#include <L298N.h>

#include <STM32RTC.h>

#include <AccelStepper.h>

#include "SdFat.h"
#include "sdios.h"



// Select your modem:
#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
HardwareSerial Serial1(PG9, PG14);
#define SerialAT Serial1

// Increase RX buffer to capture the entire response
// Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 1024
#endif

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// #define LOGGING  // <- Logging is for the HTTP library

// Range to attempt to autobaud
// NOTE:  DO NOT AUTOBAUD in production code.  Once you've established
// communication, set a fixed baud rate using modem.setBaud(#).
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// set GSM PIN, if any
#define GSM_PIN "9526"

// Your GPRS credentials, if any
const char apn[]      = "internet.vodafone.net";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details
const char server[]   = "water-minilab.herokuapp.com";  //nem lehet előtte http://
const int  port       = 80;

TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);
HttpClient    http(client, server, port);

/******************************************************************************/

/* Get the rtc object */
STM32RTC &rtc = STM32RTC::getInstance();

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
/* Monday 15th June 2015 */
const byte weekDay = 1;
const byte day = 15;
const byte month = 6;
const byte year = 15;


Servo servo1;
Servo servo2;


Relay relay1(RELAY_1, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed
Relay relay2(RELAY_2, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed


Relay mosfet1(MOSFET_1, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed
Relay mosfet2(MOSFET_2, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed


// Create one motor instance
L298N dc_pump_1(DC_PUMP_1_PWM, DC_PUMP_1_EN_1, DC_PUMP_1_EN_2);
L298N dc_pump_2(DC_PUMP_2_PWM, DC_PUMP_2_EN_1, DC_PUMP_2_EN_2);
//L298N dc_pump_3(DC_PUMP_3_PWM, DC_PUMP_3_EN_1, DC_PUMP_3_EN_2);
L298N dc_pump_4(DC_PUMP_4_PWM, DC_PUMP_4_EN_1, DC_PUMP_4_EN_2);


AccelStepper stepper1(AccelStepper::DRIVER, STEPPER_1_STEP, STEPPER_1_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER_3_STEP, STEPPER_3_DIR);


void button_ISR()
{
  Serial.println("Reset by USER_BUTTON");
  HAL_NVIC_SystemReset();
}

void sw1_ISR()
{
  Serial.println("SW1 interrupt");
}

void sw2_ISR()
{
  Serial.println("SW2 interrupt");
}

void sw3_ISR()
{
  Serial.println("SW3 interrupt");
}

void sw4_ISR()
{
  Serial.println("SW4 interrupt");
}

void sw5_ISR()
{
  Serial.println("SW5 interrupt");
}

void sw6_ISR()
{
  Serial.println("SW6 interrupt");
}

void sw7_ISR()
{
  Serial.println("SW7 interrupt");
}

void sw8_ISR()
{
  Serial.println("SW8 interrupt");
}
void setup()
{
  Serial.begin(115200);
  Serial.println("Water minilab");

  servo1.attach(SERVO_1_PWM);
  servo2.attach(SERVO_2_PWM);

  relay1.begin();
  relay2.begin();
  mosfet1.begin();
  mosfet2.begin();

  Serial.println("- configure RTC");
  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  rtc.setClockSource(STM32RTC::LSE_CLOCK);

  rtc.begin(); // initialize RTC 24H format

  // Set the time
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setWeekDay(weekDay);
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  // you can use also
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(weekDay, day, month, year);


  pinMode(USER_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(USER_BUTTON), button_ISR, FALLING);

  pinMode(SW1, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW1), sw1_ISR, FALLING);
  pinMode(SW2, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW2), sw2_ISR, FALLING);
  pinMode(SW3, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW3), sw3_ISR, FALLING);
  pinMode(SW4, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW4), sw4_ISR, FALLING);
  pinMode(SW5, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW5), sw5_ISR, FALLING);
  pinMode(SW6, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW6), sw6_ISR, FALLING);
  pinMode(SW7, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW7), sw7_ISR, FALLING);
  pinMode(SW8, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW8), sw8_ISR, FALLING);

  Serial.println("- testing DC pumps");
  dc_pump_2.setSpeed(255);
  dc_pump_2.forward();
  delay(3000);

  dc_pump_2.stop();
  delay(3000);

  dc_pump_2.setSpeed(127);
  dc_pump_2.backward();
  delay(3000);
  dc_pump_2.stop();
 

  Serial.println("- testing relays");
	relay1.turnOff(); //turns relay off
  relay2.turnOff(); //turns relay off
  delay(1000);
	relay1.turnOn();  //turns relay on
  relay2.turnOn();  //turns relay on
  delay(1000);
  relay1.turnOff(); //turns relay off
  relay2.turnOff(); //turns relay off
  delay(1000);
	relay1.turnOn();  //turns relay on
  relay2.turnOn();  //turns relay on

  Serial.println("- testing MOSFETS");
	mosfet1.turnOff(); //turns relay off
  mosfet2.turnOff(); //turns relay off
  delay(1500);
	mosfet1.turnOn();  //turns relay on
  mosfet2.turnOn();  //turns relay on
  delay(1500);
  mosfet1.turnOff(); //turns relay off
  mosfet2.turnOff(); //turns relay off

  Serial.println("- testing servos");
  for (int i = 0; i < 180; i++) {
    servo1.write(i);
    servo2.write(i);
    delay(3);
  }
  for (int i = 180; i > 0; i--) {
    servo1.write(i);
    servo2.write(i);
    delay(3);
  }

  Serial.println("- testing steppers");
  stepper1.setMaxSpeed(3000.0);
  stepper1.setAcceleration(3000.0);
  stepper1.moveTo(100);

  // stepper3.setMaxSpeed(3000.0);
  // stepper3.setAcceleration(3000.0);
  // stepper3.moveTo(100);

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  Serial.println("- testing GSM HTTP GET and POST");

  
  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!
  pinMode(MCU_GSM_PWRKEY, OUTPUT);

  digitalWrite(MCU_GSM_PWRKEY, HIGH);
  delay(1000); //Need delay
  digitalWrite(MCU_GSM_PWRKEY, LOW);

  SerialMon.println("Wait...");

  // Set GSM module baud rate
  TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }

  SerialMon.print(F("Performing HTTP GET request... "));
  int err = http.get("/binarysize");
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    delay(10000);
    return;
  }

  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    delay(10000);
    return;
  }

  SerialMon.println(F("Response Headers:"));
  while (http.headerAvailable()) {
    String headerName  = http.readHeaderName();
    String headerValue = http.readHeaderValue();
    SerialMon.println("    " + headerName + " : " + headerValue);
  }

  int length = http.contentLength();
  if (length >= 0) {
    SerialMon.print(F("Content length is: "));
    SerialMon.println(length);
  }
  if (http.isResponseChunked()) {
    SerialMon.println(F("The response is chunked"));
  }

  String body = http.responseBody();
  SerialMon.println(F("Response:"));
  SerialMon.println(body);

  SerialMon.print(F("Body length is: "));
  SerialMon.println(body.length());

  /*********************************************************************/
  // HTTP POST request
  SerialMon.print(F("\r\nPerforming HTTP POST request... "));

  http.connectionKeepAlive();
  http.beginRequest();
  int codePost = http.post("/upload");

  http.sendHeader(F("Content-Type"), F("application/json"));
  http.sendHeader(F("Content-Length"), 13);
  http.beginBody();
  http.println("{\"num\": \"15\"}");
  
  http.endRequest();

  status = http.responseStatusCode();
  //int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  /*********************************************************************/

  // Shutdown
  http.stop();
  SerialMon.println(F("Server disconnected"));

  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));

  // Do nothing forevermore
  while (true) { delay(1000); }
}


void loop()
{
  // // Print date...
  // Serial.printf("%02d/%02d/%02d ", rtc.getDay(), rtc.getMonth(), rtc.getYear());
  // // ...and time
  // Serial.printf("%02d:%02d:%02d.%03d\n", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getSubSeconds());
  // delay(1000);

  // Serial.println("- testing DC pumps");
  // dc_pump_2.setSpeed(255);
  // dc_pump_2.forward();
  // delay(3000);

  // dc_pump_2.stop();
  // delay(3000);

  // dc_pump_2.setSpeed(127);
  // dc_pump_2.backward();
  // delay(3000);
  // dc_pump_2.stop();

  // Change direction at the limits
  // if (stepper1.distanceToGo() == 0)
  // {
  //   stepper1.moveTo(-stepper1.currentPosition());
  // }
  // stepper1.run();

}

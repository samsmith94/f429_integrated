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
#include <CRC32.h>                //unzipLIB-el probléma lesz!!!

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

const char zip_resource[]    = "/download";
uint32_t   knownCRC32    = 0xa26a7a0a;
uint32_t   knownFileSize = 22820 ;  // In case server does not send it


TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);
HttpClient    http(client, server, port);

/******************************************************************************/

SPIClass SPI_4(SD_CARD_MOSI, SD_CARD_MISO, SD_CARD_SCK); // MOSI, MISO. SCLK

const uint8_t SD_CS_PIN = SD_CARD_CS;

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &SPI_4)


SdFat sd;
File file;
File root;


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

void printPercent(uint32_t readLength, uint32_t contentLength) {
  // If we know the total length
  if (contentLength != (uint32_t)-1) {
    SerialMon.print("\r ");
    SerialMon.print((100.0 * readLength) / contentLength);
    SerialMon.print('%');
  } else {
    SerialMon.println(readLength);
  }
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

  //sajnos nem igazán szereti ha közben a http megy :D
  // pinMode(SW1, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW1), sw1_ISR, FALLING);
  // pinMode(SW2, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW2), sw2_ISR, FALLING);
  // pinMode(SW3, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW3), sw3_ISR, FALLING);
  // pinMode(SW4, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW4), sw4_ISR, FALLING);
  // pinMode(SW5, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW5), sw5_ISR, FALLING);
  // pinMode(SW6, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW6), sw6_ISR, FALLING);
  // pinMode(SW7, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW7), sw7_ISR, FALLING);
  // pinMode(SW8, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SW8), sw8_ISR, FALLING);

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

  // Serial.println("- testing steppers");
  // stepper1.setMaxSpeed(3000.0);
  // stepper1.setAcceleration(3000.0);
  // stepper1.moveTo(100);
  
  
  Serial.println("- testing SD card");
  // Initialize the SD card.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }

  if (!root.open("/"))
  {
    Serial.println("Open root failed");
  }

  //először el kell távolítani a már létező fájlokat!!!, emiatt nem működött először
  if (sd.exists("Folder1"))
  {
    if (sd.exists("Folder1/file1.txt"))
    {
      Serial.println("Folder1/file1.txt is already existing");

      // Change volume working directory to Folder1.
      if (!sd.chdir("Folder1"))
      {
        Serial.println("chdir failed for Folder1.");
      }
      Serial.println("chdir to Folder1");

      // Remove files from current directory.
      if (!sd.remove("file1.txt"))
      {
        Serial.println("remove failed");
      }
      Serial.println("file1.txt removed.");

      // Change current directory to root.
      if (!sd.chdir())
      {
        Serial.println("chdir to root failed.");
      }

      // Remove Folder1.
      if (!sd.rmdir("Folder1"))
      {
        Serial.println("rmdir for Folder1 failed");
      }
      Serial.println("Folder1 removed.");
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  // Create a new folder.
  if (!sd.mkdir("Folder1"))
  {
    Serial.println("Create Folder1 failed");
  }

  if (!sd.mkdir("Folder2"))
  {
    Serial.println("Create Folder2 failed");
  }

  // Create a file in Folder1 using a path.
  if (!file.open("Folder1/file1.txt", O_WRONLY | O_CREAT))
  {
    Serial.println("Create Folder1/file1.txt failed");
  }
  Serial.println("Created Folder1/file1.txt");
  file.println("Hello 1, 2, 3");
  Serial.println("Hello 1, 2, 3 written to Folder1/file1.txt");

  Serial.print("Size of Folder1/file1.txt: ");
  Serial.println(file.fileSize());

  file.close();
  Serial.println("Folder1/file1.txt closed");
  Serial.println("****************************************");

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

  /*********************************************************************/
  // NIST time
  modem.NTPServerSync("132.163.96.5", 20);

  int   year3    = 0;
  int   month3   = 0;
  int   day3     = 0;
  int   hour3    = 0;
  int   min3     = 0;
  int   sec3     = 0;
  float timezone = 0;
  for (int8_t i = 5; i; i--) {
    SerialMon.println("Requesting current network time");
    if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,
                             &timezone)) {
      // DBG("Year:", year3, "\tMonth:", month3, "\tDay:", day3);
      // DBG("Hour:", hour3, "\tMinute:", min3, "\tSecond:", sec3);
      // DBG("Timezone:", timezone);
      break;
    } else {
      SerialMon.println("Couldn't get network time, retrying in 15s.");
      delay(15000L);
    }
  }
  SerialMon.println("Retrieving time again as a string");
  String time = modem.getGSMDateTime(DATE_FULL);
  SerialMon.print("Current Network Time:"); SerialMon.println(time);

  /*********************************************************************/
  // HTTP download zip

  SerialMon.print(F("Connecting to "));
  SerialMon.print(server);
  if (!client.connect(server, port)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  // Make a HTTP GET request:
  client.print(String("GET ") + zip_resource + " HTTP/1.0\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");

  // Let's see what the entire elapsed time is, from after we send the request.
  uint32_t timeElapsed = millis();

  SerialMon.println(F("Waiting for response header"));

  // While we are still looking for the end of the header (i.e. empty line
  // FOLLOWED by a newline), continue to read data into the buffer, parsing each
  // line (data FOLLOWED by a newline). If it takes too long to get data from
  // the client, we need to exit.

  const uint32_t clientReadTimeout   = 5000;
  uint32_t       clientReadStartTime = millis();
  String         headerBuffer;
  bool           finishedHeader = false;
  uint32_t       contentLength  = 0;

  while (!finishedHeader) {
    int nlPos;

    if (client.available()) {
      clientReadStartTime = millis();
      while (client.available()) {
        char c = client.read();
        headerBuffer += c;

        // Uncomment the lines below to see the data coming into the buffer
        // if (c < 16)
        //   SerialMon.print('0');
        // SerialMon.print(c, HEX);
        // SerialMon.print(' ');
        // if (isprint(c))
        //   SerialMon.print(reinterpret_cast<char> c);
        // else
        //   SerialMon.print('*');
        // SerialMon.print(' ');

        // Let's exit and process if we find a new line
        if (headerBuffer.indexOf(F("\r\n")) >= 0) break;
      }
    } else {
      if (millis() - clientReadStartTime > clientReadTimeout) {
        // Time-out waiting for data from client
        SerialMon.println(F(">>> Client Timeout !"));
        break;
      }
    }

    // See if we have a new line.
    nlPos = headerBuffer.indexOf(F("\r\n"));

    if (nlPos > 0) {
      headerBuffer.toLowerCase();
      // Check if line contains content-length
      if (headerBuffer.startsWith(F("content-length:"))) {
        contentLength =
            headerBuffer.substring(headerBuffer.indexOf(':') + 1).toInt();
        // SerialMon.print(F("Got Content Length: "));  // uncomment for
        // SerialMon.println(contentLength);            // confirmation
      }

      headerBuffer.remove(0, nlPos + 2);  // remove the line
    } else if (nlPos == 0) {
      // if the new line is empty (i.e. "\r\n" is at the beginning of the line),
      // we are done with the header.
      finishedHeader = true;
    }
  }

  // The two cases which are not managed properly are as follows:
  // 1. The client doesn't provide data quickly enough to keep up with this
  // loop.
  // 2. If the client data is segmented in the middle of the 'Content-Length: '
  // header,
  //    then that header may be missed/damaged.
  //

  uint32_t readLength = 0;
  CRC32    crc;

  if (finishedHeader && contentLength == knownFileSize) {
    SerialMon.println(F("Reading response data"));
    clientReadStartTime = millis();

    printPercent(readLength, contentLength);
    while (readLength < contentLength && client.connected() &&
           millis() - clientReadStartTime < clientReadTimeout) {
      while (client.available()) {
        uint8_t c = client.read();
        // SerialMon.print(reinterpret_cast<char>c);  // Uncomment this to show
        // data
        crc.update(c);
        readLength++;
        if (readLength % (contentLength / 13) == 0) {
          printPercent(readLength, contentLength);
        }
        clientReadStartTime = millis();
      }
    }
    printPercent(readLength, contentLength);
  }

  timeElapsed = millis() - timeElapsed;
  SerialMon.println();

  // Shutdown

  //client.stop();
  //SerialMon.println(F("Server disconnected"));

  //modem.gprsDisconnect();
  //SerialMon.println(F("GPRS disconnected"));

  float duration = float(timeElapsed) / 1000;

  SerialMon.println();
  SerialMon.print("Content-Length: ");
  SerialMon.println(contentLength);
  SerialMon.print("Actually read:  ");
  SerialMon.println(readLength);
  SerialMon.print("Calc. CRC32:    0x");
  SerialMon.println(crc.finalize(), HEX);
  SerialMon.print("Known CRC32:    0x");
  SerialMon.println(knownCRC32, HEX);
  SerialMon.print("Duration:       ");
  SerialMon.print(duration);
  SerialMon.println("s");

  /*********************************************************************/
  // HTTP GET request

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

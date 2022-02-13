#include <Arduino.h>
#include <pins.h>

#include <Relay.h>
#include <MOSFET.h>

#include <Servo.h>
#include <L298N.h>
#include <STM32RTC.h>
#include <AccelStepper.h>

#include <HexDump.h>
#include <unzipLIB.h>
#include "SdFat.h"
#include "sdios.h"

// Select your modem:
#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <CRC32.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>
#include <DebugLog.h>

/******************************************************************************
 ** Global variables and functions for DebugLog *******************************
 ******************************************************************************/
SdFat fs;

void shorten(String& s) {
    for (size_t i = 0; i < s.length(); ++i) {
        if (s[i] == ':')
            s.setCharAt(i, '_');
    }
}

/******************************************************************************
 ** Defies, global variables for unzipLIB *************************************
 ******************************************************************************/
#define BUFF_SIZE 1024
static uint8_t l_Buff[BUFF_SIZE];

SPIClass SPI_4(PE6, PE5, PE2); // MOSI, MISO. SCLK
const uint8_t SD_CS_PIN = PE4;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &SPI_4)

UNZIP zip; // statically allocate the UNZIP structure (41K)
SdFat sd;
File root;

static File myfile;
static File myfileAnother;
static File binFile;
static File downloadedZIPFile;

void * myOpen(const char *filename, int32_t *size) {
  root.open("/");
  myfileAnother.open(filename);
  *size = (uint32_t)myfileAnother.fileSize();
  return (void *)&myfileAnother;
}

void myClose(void *p) {
  ZIPFILE *pzf = (ZIPFILE *)p;
  File *f = (File *)pzf->fHandle;
  if (f) f->close();
}

int32_t myRead(void *p, uint8_t *buffer, int32_t length) {
  ZIPFILE *pzf = (ZIPFILE *)p;
  File *f = (File *)pzf->fHandle;
  return f->read(buffer, length);
}

int32_t mySeek(void *p, int32_t position, int iType) {
  ZIPFILE *pzf = (ZIPFILE *)p;
  File *f = (File *)pzf->fHandle;
  if (iType == SEEK_SET)
    return f->seek(position);
  else if (iType == SEEK_END) {
    return f->seek(position + pzf->iSize); 
  } else { // SEEK_CUR
    long l = f->position();
    return f->seek(l + position);
  }
}

/******************************************************************************
 ** Defies, global variables for TinyGSM **************************************
 ******************************************************************************/
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


//#define USE_HTTPS

#ifdef USE_HTTPS
#define TINY_GSM_SSL_CLIENT_AUTHENTICATION
#endif

//for MQTT (led toggle)
#define LED_PIN BLUE_LED

// set GSM PIN, if any
#define GSM_PIN               "9526"

// Set phone numbers, if you want to test SMS and Calls
#define SMS_TARGET            "+36706347173"
#define CALL_TARGET           "+36706347173"

// Your GPRS credentials, if any
#define APN                   "internet.vodafone.net"
#define GPRS_USER             ""
#define GPRS_PASSWORD         ""

// HTTP server details
// Never prepend http://
#define HTTP_SERVER           "water-minilab.herokuapp.com"
#if defined(USE_HTTPS)
#define HTTP_PORT             443
#else
#define HTTP_PORT             80
#endif

#define ZIP_RESOURCE_ENDPOINT "/download"
#define ZIP_RESOURCE_CRC32    0xDC6DD831
#define ZIP_RESOURCE_FILESIZE 50338

#define BINARY_SIZE_ENDPOINT  "/binarysize"

#define POST_DATA_ENDPOINT    "/upload"

// MQTT details
#define MQTT_BROKER           "broker.hivemq.com"

#define MQTT_TOPIC_LED        "GsmClientTest/led"
#define MQTT_TOPIC_INIT       "GsmClientTest/init"
#define MQTT_TOPIC_LEDSTATUS  "GsmClientTest/ledStatus"

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;

// for HTTP
TinyGsm       modem(SerialAT);


#if defined(USE_HTTPS)
TinyGsmClientSecure http_client(modem);
#else
TinyGsmClient http_client(modem, 1);
#endif
HttpClient    http(http_client, HTTP_SERVER, HTTP_PORT);

// for MQTT
TinyGsmClient mqtt_client(modem, 2);
PubSubClient  mqtt(mqtt_client);


void printPercent(uint32_t readLength, uint32_t contentLength) {
  // If we know the total length
  if (contentLength != (uint32_t)-1) {
    SerialMon.print("\r ");
    SerialMon.print((100.0 * readLength) / contentLength);
    SerialMon.println('%');
  } else {
    SerialMon.println(readLength);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == MQTT_TOPIC_LED) {
    ledStatus = !ledStatus;
    digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(MQTT_TOPIC_LEDSTATUS, ledStatus ? "1" : "0");
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(MQTT_BROKER);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  mqtt.publish(MQTT_TOPIC_INIT, "GsmClientTest started");
  mqtt.subscribe(MQTT_TOPIC_LED);
  return mqtt.connected();
}

/******************************************************************************
 ** Defies, global variables for STM32RTC *************************************
 ******************************************************************************/
/* Get the rtc object */
STM32RTC &rtc = STM32RTC::getInstance();

// /* Change these values to set the current initial time */
// const byte seconds = 0;
// const byte minutes = 0;
// const byte hours = 16;

// /* Change these values to set the current initial date */
// /* Monday 15th June 2015 */
// const byte weekDay = 1;
// const byte day = 15;
// const byte month = 6;
// const byte year = 15;

const int dayofweek(int year, int month, int day) {
  /* using C99 compound literals in a single line: notice the splicing */
  return ((const int [])                                         \
          {1, 2, 3, 4, 5, 6, 0})[           \
      (                                                            \
          day                                                      \
        + ((153 * (month + 12 * ((14 - month) / 12) - 3) + 2) / 5) \
        + (365 * (year + 4800 - ((14 - month) / 12)))              \
        + ((year + 4800 - ((14 - month) / 12)) / 4)                \
        - ((year + 4800 - ((14 - month) / 12)) / 100)              \
        + ((year + 4800 - ((14 - month) / 12)) / 400)              \
        - 32045                                                    \
      ) % 7];
}

/******************************************************************************
 ** Defies, global variables for Servo, Relay, MOSFET, L298N and AccelStepper *
 ******************************************************************************/
Servo servo1;
Servo servo2;

Relay relay1(RELAY_1, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed
Relay relay2(RELAY_2, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed

MOSFET mosfet1(MOSFET_1, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed
MOSFET mosfet2(MOSFET_2, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed

// Create one motor instance
L298N dc_pump_1(DC_PUMP_1_PWM, DC_PUMP_1_EN_1, DC_PUMP_1_EN_2);
L298N dc_pump_2(DC_PUMP_2_PWM, DC_PUMP_2_EN_1, DC_PUMP_2_EN_2);
//L298N dc_pump_3(DC_PUMP_3_PWM, DC_PUMP_3_EN_1, DC_PUMP_3_EN_2);
L298N dc_pump_4(DC_PUMP_4_PWM, DC_PUMP_4_EN_1, DC_PUMP_4_EN_2);

AccelStepper stepper1(AccelStepper::DRIVER, STEPPER_1_STEP, STEPPER_1_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER_3_STEP, STEPPER_3_DIR);

/******************************************************************************
 ** Defies, global variables limit swithces ***********************************
 ******************************************************************************/
void user_button_ISR()
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

/******************************************************************************
 ** setup() *******************************************************************
 ******************************************************************************/
void setup()
{
  Serial.begin(115200);
  Serial.println("Water minilab");

  //////////////////////////////////////////////////////////////////////////////
  // TESTING FILE LOGGER ///////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  Serial.println("- testing file logger");
  // Esetleg HTTP-n el is kéne majd küldeni a log-ot ha valami gond van.
  if (fs.begin(SD_CONFIG)) {
      PRINTLN("FileSystem initialization success");

      String filename = "/" + String(__TIME__) + ".txt";   // Ezt nyilván majd az RTC-ből kéne.
      shorten(filename);

      // Set file system to save log manually
      LOG_ATTACH_FS_MANUAL(fs, filename, FILE_WRITE);  // overwrite file
      // LOG_ATTACH_FS_MANUAL(fs, filename, FILE_APPEND);  // append to file

  } else {
      ASSERTM(false, "FileSystem initialization failed!");
  }

  // PRINT_FILE and PRINTLN_FILE is not affected by file_level (always visible)
  // PRINT_FILE and PRINTLN_FILE is not displayed to Serial
  PRINT_FILE("DebugLog", "can print variable args: ");
  PRINTLN_FILE(1, 2.2, "three", "=> like this");

  // Apart from the log level to be displayed,
  // you can set the log level to be saved to a file (Default is DebugLogLevel::LVL_ERROR)
  LOG_FILE_SET_LEVEL(DebugLogLevel::LVL_ERROR);
  //PRINTLN_FILE("current log level is", (int)LOG_FILE_GET_LEVEL());
  LOG_INFO("current log level is", (int)LOG_FILE_GET_LEVEL());

  // The default log_leval is DebugLogLevel::LVL_INFO
  // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
  // PRINTLN_FILE("current file level is", (int)LOG_FILE_GET_LEVEL());

  // LOG_XXXX outpus both Serial and File based on log_level and file_level
  // The default log_leval is DebugLogLevel::LVL_INFO
  // The default file_leval is DebugLogLevel::LVL_ERROR
  LOG_ERROR("this is error log");  // printed to both Serial and File
  LOG_WARN("this is warn log");    // won't be saved but printed
  LOG_INFO("this is info log");    // won't be saved but printed
  LOG_DEBUG("this is debug log");  // won't be printed
  LOG_TRACE("this is trace log");  // won't be printed

  // Log array
  float arr[3] {1.1, 2.2, 3.3};
  PRINTLN_FILE("Array can be also printed like this", LOG_AS_ARR(arr, 3));

  LOG_FILE_FLUSH();  // save to SD card and continue logging
  LOG_FILE_CLOSE();  // flush() and finish logging (ASSERT won't be saved to SD)

  delay(1000);

#define ENABLE_TEST_ASSERTION 0
#if ENABLE_TEST_ASSERTION
  // If assertion failed, suspend program after prints message and close files
  // assertions are automatically saved if DebugLog is not closed
  // if DebugLog is closed, assertions won't be saved to SD
  int x = 1;
  // ASSERT(x != 1);
  // You can also use assert with messages by ASSERTM macro
  ASSERTM(x != 1, "This always fails");

  if (LOG_FILE_IS_OPEN()) {
      LOG_FILE_CLOSE();
  }
  PRINTLN("if DEBUGLOG_DISABLE_LOG is commented out (assert is enabled), does not come here");
#endif

  //////////////////////////////////////////////////////////////////////////////
  // TESTING SERVOS, RELAYS, MOSFETS, DC PUMPS, STEPPER MOTORS /////////////////
  //////////////////////////////////////////////////////////////////////////////
  servo1.attach(SERVO_1_PWM);
  servo2.attach(SERVO_2_PWM);

  relay1.begin();
  relay2.begin();
  mosfet1.begin();
  mosfet2.begin();
  
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
	relay1.turnOff();
  relay2.turnOff();
  delay(1000);
	relay1.turnOn();
  relay2.turnOn();
  delay(1000);
  relay1.turnOff();
  relay2.turnOff();
  delay(1000);
	relay1.turnOn();
  relay2.turnOn();

  Serial.println("- testing MOSFETS");
	mosfet1.turnOff();
  mosfet2.turnOff();
  delay(1500);
	mosfet1.turnOn();
  mosfet2.turnOn();
  delay(1500);
  mosfet1.turnOff();
  mosfet2.turnOff();

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

#if 0
  Serial.println("- testing steppers");
  stepper1.setMaxSpeed(3000.0);
  stepper1.setAcceleration(3000.0);
  stepper1.moveTo(100);
  while(1) {
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
    {
      stepper1.moveTo(-stepper1.currentPosition());
    }
    stepper1.run();
  }
#endif

  //////////////////////////////////////////////////////////////////////////////
  // TESTING LIMIT SWITCH INTERRUPTS ///////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // pinMode(USER_BUTTON, INPUT);
  // attachInterrupt(digitalPinToInterrupt(USER_BUTTON), user_button_ISR, FALLING);

#define ENABLE_TEST_INTERRUPTS 0
#if ENABLE_TEST_INTERRUPTS
  // Sajnos nem igazán szereti ha közben a HTTP megy. Le kall majd tiltani a megszakításokat: detachInterrupt(digitalPinToInterrupt(SW1))
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
#endif

  //////////////////////////////////////////////////////////////////////////////
  // TESTING SD CARD DIRECTORY FUNCTIONS ///////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
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
  if (!myfile.open("Folder1/file1.txt", O_WRONLY | O_CREAT))
  {
    Serial.println("Create Folder1/file1.txt failed");
  }
  Serial.println("Created Folder1/file1.txt");
  myfile.println("Hello 1, 2, 3");
  Serial.println("Hello 1, 2, 3 written to Folder1/file1.txt");

  Serial.print("Size of Folder1/file1.txt: ");
  Serial.println(myfile.fileSize());

  myfile.close();
  Serial.println("Folder1/file1.txt closed");
  Serial.println("****************************************");

  //////////////////////////////////////////////////////////////////////////////
  // TESTING ALL GSM FUNCTIONS (HTTP GET/POST & MQTT, SMS, NETWORK TIME) ///////
  //////////////////////////////////////////////////////////////////////////////
  Serial.println("- testing GSM HTTP GET and POST");

  DynamicJsonDocument doc(64);    // Sokat számít a méret! 1024-el timeout volt!
  
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

  // WAITING FOR NETWORK ///////////////////////////////////////////////////////
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

  // CONNECT ///////////////////////////////////////////////////////////////////
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(APN);
  if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASSWORD)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }

  // MQTT //////////////////////////////////////////////////////////////////////
  // MQTT Broker setup
  mqtt.setServer(MQTT_BROKER, 1883);
  mqtt.setCallback(mqttCallback);

  pinMode(LED_PIN, OUTPUT);

  // SMS ///////////////////////////////////////////////////////////////////////
#define ENABLE_TEST_SMS 0
#if ENABLE_TEST_SMS
  String imei = modem.getIMEI();
  bool res = modem.sendSMS(SMS_TARGET, String("Hello from ") + imei);
  SerialMon.print("SMS: ");
  if (res)
  {
    SerialMon.println("OK");
  } else {
    SerialMon.println("fail");
  }
#endif
  // NIST TIME /////////////////////////////////////////////////////////////////
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
    if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3, &timezone)) {
      break;
    } else {
      SerialMon.println("Couldn't get network time, retrying in 15s.");
      delay(15000L);
    }
  }
  SerialMon.println("Retrieving time again as a string");
  String time = modem.getGSMDateTime(DATE_FULL);
  SerialMon.print("Current Network Time:"); SerialMon.println(time);

  // HTTP ZIP DOWNLOAD /////////////////////////////////////////////////////////
  // Create a application.bin file to write RAM to SD card
  if (!downloadedZIPFile.open("application.zip", O_WRONLY | O_CREAT))
  {
    Serial.println("Create application.zip failed");
  }
  else
  {
    Serial.println("application.zip created");
  }


  SerialMon.print(F("Connecting to "));
  SerialMon.print(HTTP_SERVER);
  if (!http_client.connect(HTTP_SERVER, HTTP_PORT)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  // Make a HTTP GET request:
  http_client.print(String("GET ") + ZIP_RESOURCE_ENDPOINT + " HTTP/1.0\r\n");
  http_client.print(String("Host: ") + HTTP_SERVER + "\r\n");
  http_client.print("Connection: close\r\n\r\n");

  // Let's see what the entire elapsed time is, from after we send the request.
  uint32_t timeElapsed = millis();

  SerialMon.println(F("Waiting for response header"));

  // While we are still looking for the end of the header (i.e. empty line
  // FOLLOWED by a newline), continue to read data into the buffer, parsing each
  // line (data FOLLOWED by a newline). If it takes too long to get data from
  // the client, we need to exit.

  const uint32_t clientReadTimeout   = 15000;
  uint32_t       clientReadStartTime = millis();
  String         headerBuffer;
  bool           finishedHeader = false;
  uint32_t       contentLength  = 0;

  while (!finishedHeader) {
    int nlPos;

    if (http_client.available()) {
      clientReadStartTime = millis();
      while (http_client.available()) {
        char c = http_client.read();
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

  if (finishedHeader && contentLength == ZIP_RESOURCE_FILESIZE) {
    SerialMon.println(F("Reading response data"));
    clientReadStartTime = millis();

    printPercent(readLength, contentLength);
    while (readLength < contentLength && http_client.connected() &&
           millis() - clientReadStartTime < clientReadTimeout) {
      while (http_client.available()) {
        uint8_t c = http_client.read();
        downloadedZIPFile.write(c);
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
  //http_client.stop();
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
  SerialMon.println(ZIP_RESOURCE_CRC32, HEX);
  SerialMon.print("Duration:       ");
  SerialMon.print(duration);
  SerialMon.println("s");

  downloadedZIPFile.close();
  
  // HTTP GET REQUEST //////////////////////////////////////////////////////////
  SerialMon.print(F("Performing HTTP GET request... "));
  int err = http.get(BINARY_SIZE_ENDPOINT);
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

  // HTTP POST REQUEST /////////////////////////////////////////////////////////
  delay(1000);
  doc["num"] = 15;
  String payload;
  serializeJson(doc, payload);
  Serial.print("POST payload: "); Serial.println(payload);
  int payloadLength = payload.length();
  Serial.print("POST payload length: "); Serial.println(payloadLength);

  SerialMon.print(F("\r\nPerforming HTTP POST request... "));

  http.connectionKeepAlive();
  http.beginRequest();
  int codePost = http.post(POST_DATA_ENDPOINT);

  http.sendHeader(F("Content-Type"), F("application/json"));
  //http.sendHeader(F("Content-Length"), 13);
  http.sendHeader(F("Content-Length"), payloadLength);
  
  http.beginBody();
  //http.println("{\"num\": \"15\"}");
  http.println(payload);

  http.endRequest();

  status = http.responseStatusCode();
  //int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  
  // SHUTDOWN AT THE END ///////////////////////////////////////////////////////
  // http.stop();
  // SerialMon.println(F("Server disconnected"));

  // modem.gprsDisconnect();
  // SerialMon.println(F("GPRS disconnected"));

  // Do nothing forevermore
  //while (true) { delay(1000); }

  // UNZIP DOWNLOADED ZIP //////////////////////////////////////////////////////  
  // Create a application.bin file to write RAM to SD card
  if (!binFile.open("application.bin", O_WRONLY | O_CREAT))
  {
    Serial.println("Create application.bin failed");
  }
  else
  {
    Serial.println("application.bin created");
  }

  
  int rc;
  char szComment[256], szName[256];
  unz_file_info fi;

  const char *name = "application.zip";
  if (!myfile.open(name, O_RDONLY))
  {
    Serial.println("Opening application.zip failed");
  }
  Serial.println("Openened application.zip");
  rc = rc = zip.openZIP(name, myOpen, myClose, myRead, mySeek);
  if (rc == UNZ_OK) {
    Serial.println("ZIP file found!");

    // Display the global comment and all of the filenames within
    rc = zip.getGlobalComment(szComment, sizeof(szComment));
    Serial.print("Files in this archive: ");
    zip.gotoFirstFile();
    rc = UNZ_OK;
    rc = zip.getFileInfo(&fi, szName, sizeof(szName), NULL, 0, szComment, sizeof(szComment));

    if (rc == UNZ_OK) {
      Serial.println(szName);
      Serial.print("Compressed size: "); Serial.println(fi.compressed_size, DEC);
      Serial.print("Uncompressed size: "); Serial.println(fi.uncompressed_size, DEC);
      
    }

    zip.locateFile("application.bin");
    zip.openCurrentFile();

    int counter = 0;
    int number_of_chunks = (fi.uncompressed_size / BUFF_SIZE);
    int rc, i;
    rc = 1;
    i = 0;
    while (rc > 0) {
        if (counter == number_of_chunks) {
          break;
        }
        rc = zip.readCurrentFile(l_Buff, BUFF_SIZE);
        binFile.write(l_Buff, BUFF_SIZE);
        counter++;
        if (rc >= 0) {
            i += rc;
        } else {
            Serial.println("Error reading from file");
            break;
        }
    }
    Serial.print("Total bytes read = ");
    Serial.println(i);
    
    zip.closeCurrentFile();
    zip.closeZIP();

    binFile.close();
    Serial.println("Now you can remove SD card.");
  }

  // TEST RTC //////////////////////////////////////////////////////////////////
  Serial.println("- configure RTC");
  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin(); // initialize RTC 24H format

  // // Set the time
  // rtc.setHours(hour3);
  // rtc.setMinutes(min3);
  // rtc.setSeconds(sec3);

  // // Set the date
  // rtc.setWeekDay(weekDay);
  // rtc.setDay(day3);
  // rtc.setMonth(month3);
  // rtc.setYear(year3);

  Serial.print("YEAR: "); Serial.println(year3);
  uint8_t weekDay = dayofweek(year3, month3, day3);
  Serial.print("weekDay = "); Serial.println(weekDay); //NEM JÓL MŰKÖDIK EZ A WEEKDAY
  // you can use also
  rtc.setTime(hour3, min3, sec3);
  rtc.setDate(weekDay, day3, month3, year3-2000);
  
  for (int i = 0; i < 10; i++) {
    // Print date...
    Serial.printf("%02d/%02d/%02d ", rtc.getYear(), rtc.getMonth(), rtc.getDay());
    // ...and time
    Serial.printf("%02d:%02d:%02d\n", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    delay(1000);
  }
}

/******************************************************************************
 ** loop() ********************************************************************
 ******************************************************************************/
void loop()
{
  // MQTT LOOP
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(APN);
      if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASSWORD)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) { SerialMon.println("GPRS reconnected"); }
    }
  }

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
  }

  mqtt.loop();
}

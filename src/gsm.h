#ifndef _GSM_h
#define _GSM_h

// #include "pins.h"

// #define TINY_GSM_MODEM_SIM800

// #include <TinyGsmClient.h>
// #include <ArduinoHttpClient.h>
// #include <CRC32.h>
// #include <PubSubClient.h>

// // Set serial for debug console (to the Serial Monitor, default speed 115200)
// #define SerialMon Serial

// // Set serial for AT commands (to the module)
// HardwareSerial Serial1(MCU_UART_RX, MCU_UART_TX);

// #define SerialAT Serial1

// // Increase RX buffer to capture the entire response
// // Chips without internal buffering (A6/A7, ESP8266, M590)
// // need enough space in the buffer for the entire response
// // else data will be lost (and the http library will fail).
// #if !defined(TINY_GSM_RX_BUFFER)
// #define TINY_GSM_RX_BUFFER 1024
// #endif

// // Define the serial console for debug prints, if needed
// #define TINY_GSM_DEBUG SerialMon
// // #define LOGGING  // <- Logging is for the HTTP library

// // Range to attempt to autobaud
// // NOTE:  DO NOT AUTOBAUD in production code.  Once you've established
// // communication, set a fixed baud rate using modem.setBaud(#).
// #define GSM_AUTOBAUD_MIN 9600
// #define GSM_AUTOBAUD_MAX 115200

// // Add a reception delay, if needed.
// // This may be needed for a fast processor at a slow baud rate.
// // #define TINY_GSM_YIELD() { delay(2); }


// class SIM800_GSM_Module
// {
// protected:
//     // for HTTP
//     TinyGsm       &modem;
//     TinyGsmClient *http_client;
//     HttpClient    *http;

//     // for MQTT
//     TinyGsmClient *mqtt_client;
//     PubSubClient  *mqtt;
// public:
//     SIM800_GSM_Module(const char* aServerName, uint16_t aServerPort);
//     void init();

// };

#endif
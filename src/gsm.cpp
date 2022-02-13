#include "gsm.h"






// SIM800_GSM_Module::SIM800_GSM_Module(const char* aServerName, uint16_t aServerPort) {
//     modem = new TinyGsm(SerialAT);
//     http_client = new TinyGsmClient(modem, 1);
//     http = new HttpClient(http_client, aServerName, aServerPort);

//     mqtt_client = new TinyGsmClient(modem, 2);
//     mqtt = new PubSubClient(mqtt_client);
// }

// void SIM800_GSM_Module::init() {
//     return;
// }


// TinyGsm       modem(SerialAT);
// TinyGsmClient http_client(modem, 1);
// HttpClient    http(http_client, HTTP_SERVER, HTTP_PORT);

// // for MQTT
// TinyGsmClient mqtt_client(modem, 2);
// PubSubClient  mqtt(mqtt_client);
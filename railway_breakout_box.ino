#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>

#include <MqttClient.h>

#ifdef ESP8266
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#elif defined(ESP32)
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#endif
#include <EEPROM.h>  // For storing the firmware version

#include <FS.h>
#include <LittleFS.h>
#include <AsyncFsWebServer.h>  // https://github.com/cotestatnt/async-esp-fs-webserver/

#define FILESYSTEM LittleFS
AsyncFsWebServer server(80, FILESYSTEM);

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#define LOG_PRINTFLN(fmt, ...) logfln(fmt, ##__VA_ARGS__)
#define LOG_SIZE_MAX 128
void logfln(const char* fmt, ...) {
  char buf[LOG_SIZE_MAX];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, LOG_SIZE_MAX, fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

#define HW_UART_SPEED 115200L
#define MQTT_ID "raysrailway"
const char* MQTT_TOPIC_SUB = MQTT_ID "/track/turnout/1";

static MqttClient* mqtt = NULL;
static WiFiClient network;

// ============== Object to supply system functions ============================
class System : public MqttClient::System {
public:

  unsigned long millis() const {
    return ::millis();
  }

  void yield(void) {
    ::yield();
  }
};


// In order to set SSID and password open the /setup webserver page
// const char* ssid;
// const char* password;

uint8_t ledPin = LED_BUILTIN;
bool apMode = false;

#ifdef ESP8266
String fimwareInfo = "https://raw.githubusercontent.com/cotestatnt/async-esp-fs-webserver/master/examples/remoteOTA/version-esp8266.json";
#elif defined(ESP32)
String fimwareInfo = "https://raw.githubusercontent.com/cotestatnt/async-esp-fs-webserver/master/examples/remoteOTA/version-esp32.json";
#endif

char fw_version[10] = { "0.0.0" };

//////////////////////////////  Firmware update /////////////////////////////////////////
void doUpdate(const char* url, const char* version) {

#ifdef ESP8266
#define UPDATER ESPhttpUpdate
#elif defined(ESP32)
#define UPDATER httpUpdate
#if ESP_ARDUINO_VERSION_MAJOR > 2
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 15 * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores
    .trigger_panic = false,
  };
  ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));
#else
  ESP_ERROR_CHECK(esp_task_wdt_init(15, 0));
#endif
#endif

  // onProgress handling is missing with ESP32 library
  UPDATER.onProgress([](int cur, int total) {
    static uint32_t sendT;
    if (millis() - sendT > 1000) {
      sendT = millis();
      Serial.printf("Updating %d of %d bytes...\n", cur, total);
    }
  });

  WiFiClientSecure client;
  client.setInsecure();
  UPDATER.rebootOnUpdate(false);
  UPDATER.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  UPDATER.setLedPin(LED_BUILTIN, LOW);
  t_httpUpdate_return ret = UPDATER.update(client, url, fw_version);
  client.stop();

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", UPDATER.getLastError(), UPDATER.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      strcpy(fw_version, version);
      EEPROM.put(0, fw_version);
      EEPROM.commit();
      Serial.print("System will be restarted with the new version ");
      Serial.println(fw_version);
      delay(1000);
      ESP.restart();
      break;
  }
}

////////////////////////////////  Filesystem  /////////////////////////////////////////
void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("\nListing directory: %s\n", dirname);
  File root = fs.open(dirname, "r");
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      if (levels) {
#ifdef ESP8266
        String path = file.fullName();
        path.replace(file.name(), "");
#elif defined(ESP32)
        String path = file.path();
#endif
        listDir(fs, path.c_str(), levels - 1);
      }
    } else {
      Serial.printf("|__ FILE: %s (%d bytes)\n", file.name(), file.size());
    }
    file = root.openNextFile();
  }
}

bool startFilesystem() {
  if (FILESYSTEM.begin()) {
    listDir(FILESYSTEM, "/", 1);
    return true;
  } else {
    Serial.println("ERROR on mounting filesystem. It will be reformatted!");
    FILESYSTEM.format();
    ESP.restart();
  }
  return false;
}


////////////////////////////  HTTP Request Handlers  ////////////////////////////////////
void handleLed(AsyncWebServerRequest* request) {
  // http://xxx.xxx.xxx.xxx/led?val=1
  if (request->hasArg("val")) {
    int value = request->arg("val").toInt();
    digitalWrite(ledPin, value);
  }

  String reply = "LED is now ";
  reply += digitalRead(ledPin) ? "OFF" : "ON";
  request->send(200, "text/plain", reply);
}

/* Handle the update request from client.
* The web page will check if is it necessary or not checking the actual version.
* Info about firmware as version and remote url, are stored in "version.json" file
*
* Using this example, the correct workflow for deploying a new firmware version is:
  - upload the new firmware.bin compiled on your web space (in this example Github is used)
  - update the "version.json" file with the new version number and the address of the binary file
  - on the update webpage, press the "UPDATE" button.
*/
void handleUpdate(AsyncWebServerRequest* request) {
  if (request->hasArg("version") && request->hasArg("url")) {
    const char* new_version = request->arg("version").c_str();
    const char* url = request->arg("url").c_str();
    String reply = "Firmware is going to be updated to version ";
    reply += new_version;
    reply += " from remote address ";
    reply += url;
    reply += "<br>Wait 10-20 seconds and then reload page.";
    request->send(200, "text/plain", reply);
    Serial.println(reply);
    doUpdate(url, new_version);
  }
}

///////////////////////////////////  SETUP  ///////////////////////////////////////
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(HW_UART_SPEED);
  EEPROM.begin(128);

  // Try to connect to flash stored SSID, start AP if fails after timeout
  IPAddress myIP = server.startWiFi(15000, "ESP_AP", "123456789");

  // FILESYSTEM INIT
  startFilesystem();

/*
  * Getting FS info (total and free bytes) is strictly related to
  * filesystem library used (LittleFS, FFat, SPIFFS etc etc) and ESP framework
  */
#ifdef ESP32
  server.setFsInfoCallback([](fsInfo_t* fsInfo) {
    fsInfo->fsName = "LittleFS";
    fsInfo->totalBytes = LittleFS.totalBytes();
    fsInfo->usedBytes = LittleFS.usedBytes();
  });
#endif

  // Enable ACE FS file web editor and add FS info callback function
  server.enableFsCodeEditor();

  // Add custom handlers to webserver
  server.on("/led", HTTP_GET, handleLed);
  server.on("/firmware_update", HTTP_GET, handleUpdate);

  // Add handler as lambda function (just to show a different method)
  server.on("/version", HTTP_GET, [](AsyncWebServerRequest* request) {
    server.getOptionValue("New firmware JSON", fimwareInfo);

    EEPROM.get(0, fw_version);
    if (fw_version[0] == 0xFF)  // Still not stored in EEPROM (first run)
      strcpy(fw_version, "0.0.0");
    String reply = "{\"version\":\"";
    reply += fw_version;
    reply += "\", \"newFirmwareInfoJSON\":\"";
    reply += fimwareInfo;
    reply += "\"}";
    // Send to client actual firmware version and address where to check if new firmware available
    request->send(200, "text/json", reply);
  });

  // Configure /setup page and start Web Server
  server.addOptionBox("Remote Update");
  server.addOption("New firmware JSON", fimwareInfo);

  // Start server with built-in websocket event handler
  server.init();
  Serial.print(F("ESP Web Server started on IP Address: "));
  Serial.println(WiFi.localIP());
  Serial.println(F(
    "This is \"remoteOTA.ino\" example.\n"
    "Open /setup page to configure optional parameters.\n"
    "Open /edit page to view, edit or upload example or your custom webserver source files."));

  // Setup MqttClient
  MqttClient::System* mqttSystem = new System;
  MqttClient::Logger* mqttLogger = new MqttClient::LoggerImpl<HardwareSerial>(Serial);
  MqttClient::Network* mqttNetwork = new MqttClient::NetworkClientImpl<WiFiClient>(network, *mqttSystem);
  //// Make 128 bytes send buffer
  MqttClient::Buffer* mqttSendBuffer = new MqttClient::ArrayBuffer<128>();
  //// Make 128 bytes receive buffer
  MqttClient::Buffer* mqttRecvBuffer = new MqttClient::ArrayBuffer<128>();
  //// Allow up to 2 subscriptions simultaneously
  MqttClient::MessageHandlers* mqttMessageHandlers = new MqttClient::MessageHandlersImpl<2>();
  //// Configure client options
  MqttClient::Options mqttOptions;
  ////// Set command timeout to 10 seconds
  mqttOptions.commandTimeoutMs = 10000;
  //// Make client object
  mqtt = new MqttClient(
    mqttOptions, *mqttLogger, *mqttSystem, *mqttNetwork, *mqttSendBuffer,
    *mqttRecvBuffer, *mqttMessageHandlers);
}

// ============== Subscription callback ========================================
void processMessage(MqttClient::MessageData& md) {
	const MqttClient::Message& msg = md.message;
	char payload[msg.payloadLen + 1];
	memcpy(payload, msg.payload, msg.payloadLen);
	payload[msg.payloadLen] = '\0';
	LOG_PRINTFLN(
		"Message arrived: qos %d, retained %d, dup %d, packetid %d, payload:[%s]",
		msg.qos, msg.retained, msg.dup, msg.id, payload
	);
  
  digitalWrite(ledPin, (strcmp("ON", payload) == 0));
}

///////////////////////////////////  LOOP  ///////////////////////////////////////
void loop() {
  // Check connection status
  if (!mqtt->isConnected()) {
    // Close connection if exists
    network.stop();
    // Re-establish TCP connection with MQTT broker
    LOG_PRINTFLN("Connecting");
    network.connect("test.mosquitto.org", 1883);
    if (!network.connected()) {
      LOG_PRINTFLN("Can't establish the TCP connection");
      delay(5000);
      ESP.restart();
    } else {
      LOG_PRINTFLN("Established network connection to test.mosquitto.org");
    }
    // Start new MQTT connection
    MqttClient::ConnectResult connectResult;
    // Connect
    {
      MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
      options.MQTTVersion = 4;
      options.clientID.cstring = (char*)MQTT_ID;
      options.cleansession = true;
      options.keepAliveInterval = 15;  // 15 seconds
      MqttClient::Error::type rc = mqtt->connect(options, connectResult);
      if (rc != MqttClient::Error::SUCCESS) {
        LOG_PRINTFLN("Connection error: %i", rc);
        return;
      }      else {
        LOG_PRINTFLN("Connected to MQTT broker");
      }
    }
    {
      MqttClient::Error::type rc = mqtt->subscribe(
				MQTT_TOPIC_SUB, MqttClient::QOS0, processMessage
			);
			if (rc != MqttClient::Error::SUCCESS) {
				LOG_PRINTFLN("Subscribe error: %i", rc);
				LOG_PRINTFLN("Drop connection");
				mqtt->disconnect();
				return;
			} else {
        LOG_PRINTFLN("Successfully subscribed to topic %s", MQTT_TOPIC_SUB);
      }

    }
  } else {
    {
      // Add publish here if required
    }
    // Idle for 30 seconds
    mqtt->yield(30000L);
  }
}
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include <PZEM004Tv30.h>


#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#endif

#if !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial2
#endif

#define NUM_PZEMS 3

PZEM004Tv30 pzems[NUM_PZEMS];

/* ***************************************************************
 * Uncomment USE_SOFTWARE_SERIAL in order to enable Softare serial
 *
 * Does not work for ESP32
 *****************************************************************/
//#define USE_SOFTWARE_SERIAL



#if defined(USE_SOFTWARE_SERIAL) && defined(ESP32)
    #error "Can not use SoftwareSerial with ESP32"
#elif defined(USE_SOFTWARE_SERIAL)

#include <SoftwareSerial.h>

SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
#endif

//#define WIFI_AP "E5573-MIFI-381476" surabaya
//#define WIFI_PASSWORD "12345678"
//#define WIFI_AP "modem" /
//#define WIFI_PASSWORD "12345678"
// #define WIFI_AP "MTN-MobileWiFi-E5573" //gresik
// #define WIFI_PASSWORD "3JAF6A3J"
#define WIFI_AP "crustea"
#define WIFI_PASSWORD "12345678"

#define TOKEN "LOMBA_TOKEN"

//#define TOKEN "EBII_LAMONGAN"
//#define TOKEN "uTHDviq2md1SZOKaCoX0"

#define GPIO23 23 //relay Pin 
#define GPIO2 2

#define GPIO23_PIN 14
//#define GPIO2_PIN 5

char thingsboardServer[] = "18.140.254.213";
//char thingsboardServer[] = "thingsboard.cloud";

WiFiClient wifiClient;

PubSubClient client(wifiClient);

int status = WL_IDLE_STATUS;

ThingsBoard tb(wifiClient);

unsigned long lastSend;

// We assume that all GPIOs are LOW
boolean gpioState[] = {false, false};
int i;

void powerSystem(int relayMode){

  /**
   * Fungsi untuk menghidupkan dan mematikan relay 
   * Digunakan untuk nyala mati aerator 
   * Panggil fungsi dengan parameter 0 untuk menyalakan relay dan 1 untuk mematikan relay
   */
   
  if(relayMode == 0){
    // Turn the relay switch ON 
    digitalWrite(GPIO23, LOW);// set relay pin to low 
    Serial.println("Relay ON ");
  }else if(relayMode == 1){
    // Turn the relay switch OFF 
    digitalWrite(GPIO23, HIGH);// set relay pin to HIGH
    Serial.println("Relay OFF ");
  }

}

void getAndSendData(){
  
  Serial.println("Sending data to ThingsBoard:");

}

void setup() {
  Serial.begin(115200);
  // Set output mode for all GPIO pins
  pinMode(GPIO23, OUTPUT);
  pinMode(GPIO2, OUTPUT);
  delay(10);
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  client.setCallback(on_message);
  lastSend = 0;
}

void loop() {
  if ( !client.connected()  ) {
    reconnect();
  }

  if ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
    getAndSendData();
//    tb.loop();
    lastSend = millis();
  }

  
  client.loop();
//  pzemMonitor();

}

// The callback for when a PUBLISH message is received from the server.
void on_message(const char* topic, byte* payload, unsigned int length) {

  Serial.println("On message");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(json);

  // Decode JSON request
  StaticJsonDocument<200> jsonBuffer;
  DeserializationError data = deserializeJson(jsonBuffer, (char*)json);

  if (data)
  {
    Serial.println("parseObject() failed");
    return;
  }

  // Check request method
  String methodName = String((const char*)jsonBuffer["method"]);

  if (methodName.equals("getGpioStatus")) {
    // Reply with GPIO status
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  } else if (methodName.equals("setGpioStatus")) {
    // Update GPIO status and reply
    set_gpio_status(jsonBuffer["params"]["pin"], jsonBuffer["params"]["enabled"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  }
}

String get_gpio_status() {
  // Prepare gpios JSON payload string
  StaticJsonDocument<200> jsonBuffer;
  JsonObject data = jsonBuffer.to<JsonObject>();
  data[String(GPIO23_PIN)] = gpioState[0] ? true : false;
  // data[String(GPIO2_PIN)] = gpioState[1] ? true : false;
  char payload[256];
  serializeJson(jsonBuffer, payload);
  String strPayload = String(payload);
  Serial.print("Get gpio status: ");
  Serial.println(strPayload);
  return strPayload;
}

void set_gpio_status(int pin, boolean enabled) {
  if (pin == GPIO23_PIN) {
    // Output GPIOs state
    digitalWrite(GPIO23, enabled ? HIGH : LOW);
    digitalWrite(GPIO2, enabled ? HIGH : LOW);
    // Update GPIOs state
    gpioState[0] = enabled;
  } 
//  else if (pin == GPIO2_PIN) {
//    // Output GPIOs state
//    digitalWrite(GPIO2, enabled ? HIGH : LOW);
//    // Update GPIOs state
//    gpioState[1] = enabled;
//  }
}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("ESP8266 Device", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }

  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED]" );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}

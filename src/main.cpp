#include "Arduino.h"
#include "credentials/credentials.h"
#include "SPI.h"
#include "Wire.h"
#include <time.h>

////////////////////////////////
// timer counts
////////////////////////////////
const uint64_t time_to_update_NTP_secs = 3600;
const int time_to_send_msecs = 30013 ;
const int time_to_update_Air_msecs = 4234 ;

////////////////////////////////
// Program state
////////////////////////////////
enum {
  INIT,
  WAIT,
  NTP,
  AIR,
  SEND,
  SENT,
  RECV,
  MQTT,
  SLEEP,
  ERROR
};

volatile int state = INIT;
volatile int prevState;

/* specifique TTGO lora OLED pins */

// I2C OLED Display works with SSD1306 driver
#define OLED_SDA   4
#define OLED_SCL  15
#define OLED_RST  16

// SPI LoRa Radio
#define LORA_SCK   5      // GPIO5 - SX1276 SCK
#define LORA_MISO 19      // GPIO19 - SX1276 MISO
#define LORA_MOSI 27      // GPIO27 - SX1276 MOSI
#define LORA_CS   18      // GPIO18 - SX1276 CS
#define LORA_RST  14      // GPIO14 - SX1276 RST
#define LORA_IRQ  26      // GPIO26 - SX1276 IRQ (interrupt request)

////////////////////////////////
// Interrupts & Sleep
////////////////////////////////
#include <esp_sleep.h>
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define mS_TO_S_FACTOR 1000ULL     /* Conversion factor for milli seconds to seconds */

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timerSend = NULL;
hw_timer_t *timerNTP = NULL;
hw_timer_t *timerAir = NULL;

void IRAM_ATTR onSend() {
  portENTER_CRITICAL_ISR(&mux);
  state = SEND;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onNTP() {
  portENTER_CRITICAL_ISR(&mux);
  state = NTP;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onAir() {
  portENTER_CRITICAL_ISR(&mux);
  state = AIR;
  portEXIT_CRITICAL_ISR(&mux);
}

////////////////////////////////
// DHT22
////////////////////////////////
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 25       // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht22(DHTPIN, DHTTYPE);
sensors_event_t event;
float DHTtemperature = 0;
float DHThumidity = 0;

void readDHT() {
  DHTtemperature = dht22.readTemperature();
  DHThumidity    = dht22.readHumidity();
} 

////////////////////////////////
// WiFi
////////////////////////////////
#include <WiFi.h>
#include <WiFiClient.h>

WiFiClient wifiClient; 

void onWifiEvent (system_event_id_t event, system_event_info_t info) {
    Serial.printf ("[WiFi-event] event: %d - ", event);
    switch (event) {
    case SYSTEM_EVENT_WIFI_READY:    
      Serial.printf ("WiFi ready\n"); 
      break;
    case SYSTEM_EVENT_STA_START:     
      Serial.printf ("WiFi start\n"); 
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.printf ("Connected to %s. Asking for IP address\n", info.connected.ssid);
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.printf ("Station Stop\n");
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.printf ("Lost IP\n");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.printf ("Got IP: %s\n", IPAddress (info.got_ip.ip_info.ip.addr).toString ().c_str ());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED: 
      Serial.printf ("Disconnected from SSID: %s\n", info.disconnected.ssid);
      break;
		default:
      Serial.printf ("Unknown event\n");
      break;
    }
}

bool WiFiConnect() {
  Serial.println("--> WiFi Connect");
  bool retval = false;
  if(!WiFi.isConnected()) WiFi.begin(AP_NAME, AP_PASSRHRASE);
  for (int t=0; t<1000; t++) {
    if (WiFi.isConnected()) {
      retval = true;
      break;
    }
    delay(10);
  }
  Serial.print("--> WiFi Connect End : "); Serial.println(retval ? "OK" : "FAILED");
  return retval;
}


/////////////////////////////////////////////////////
// OTA
/////////////////////////////////////////////////////
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

AsyncWebServer server(80);

void initOTA(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

////////////////////////////////
// NTP
////////////////////////////////
#include <WiFiUdp.h>
#include <NTPClient.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "fr.pool.ntp.org", 3600, 60000);

////////////////////////////////
// MQ-135
////////////////////////////////

#define MQ135_A0_PIN 34
const int RESOLUTION = 4095; // 12 bit 
float ppm;
float correctedPPM;
float rzero;
float correctedRZero;
float resistance;

const float Ve = 5.0;
const float R2 = 5040;
const float R1 = 9925;   

const float diviseur = R2 / (R1 + R2); 

#include <MQ135.h>
#define PIN_MQ135 34
float temperature = 21.0; // assume current temperature. Recommended to measure with DHT22
float humidity = 55.0; // assume current humidity. Recommended to measure with DHT22

MQ135 mq135_sensor = MQ135(PIN_MQ135);

void MQ135_read(int samples=1) {
  ppm = 0;
  rzero = mq135_sensor.getRZero();
  correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
  resistance = mq135_sensor.getResistance();
  for (int i=0 ; i < samples ; i++){
    ppm += mq135_sensor.getPPM();
    delay(2);
  }
  ppm /= samples;
  correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);

}

////////////////////////////////
// JSon
////////////////////////////////
#include <ArduinoJson.h>

//////////////////////////////////////
// LoRa 
//////////////////////////////////////
#include <SPI.h>
#include <LoRa.h>

#define LORA_BAND                                   870E6    // Hz
#define LORA_TX_POWER                               20        // dBm
#define LORA_BANDWIDTH                              125E3    
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE_DENOMINATOR                 5         // 4/5

String textSend, textRecv ;

const uint8_t LoRa_buffer_size = 128; // Define the payload size here
char txpacket[LoRa_buffer_size];


void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

/* void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
} */

void onTxDone() {
  portENTER_CRITICAL_ISR(&mux);
  state = SENT;
  portEXIT_CRITICAL_ISR(&mux);
}

void LoRa_sendMessage() {
  LoRa.beginPacket();                 // start packet
  LoRa.print(textSend);               // add payload
  LoRa.endPacket(true);               // finish packet and send it
  Serial.println("send message => "); 
  Serial.println(textSend);
}

void onReceive(int packetSize) {
  textRecv.clear();
  while (LoRa.available()) {
    textRecv += (char)LoRa.read();
  }
  portENTER_CRITICAL_ISR(&mux);
  state = RECV;
  portEXIT_CRITICAL_ISR(&mux);
}


////////////////////////////////
// Send data
////////////////////////////////

size_t prepareTxFrame(uint8_t port) {
  Serial.println("PerpareTxFrame");
  textSend.clear();
  char st[40];

  StaticJsonDocument<200> jsonDoc;
  jsonDoc["device"] = "Air";
  //jsonDoc["ppm"] = roundFloat(correctedPPM);
  sprintf(st,"%.2f",correctedPPM);
  jsonDoc["ppm"] = st; 
  sprintf(st,"%.2f",correctedRZero);
  jsonDoc["RZero"] = st;
  sprintf(st,"%.2f",resistance);
  jsonDoc["resistance"] = st;
  sprintf(st,"%.2f",temperature);
  jsonDoc["temp"] = st;
  sprintf(st,"%.2f",humidity);
  jsonDoc["humid"] = st;

  size_t size = serializeJsonPretty(jsonDoc, textSend);
  Serial.printf("prepare packet [%d]\n", textSend.length());
  return size;
}


////////////////////////////////
// MQTT
////////////////////////////////
/* #include <PubSubClient.h>

char subStrTempInt[64] = "home/cubecell/#";
char recvTopic[128];
char recvMessage[128];
float MQTTtemperature = 21.0;

PubSubClient mqttClient(wifiClient);

void receivedMQTT(char* topic, byte* payload, unsigned int length) {
  int ret = strcmp(topic,"home/cubecell/temp");
  if (ret==0) {
    MQTTtemperature = strtof((char *)payload, NULL);
    Serial.printf("MQTT temperature interieur = %.1f°C \n",temperature); 
  }
  state = MQTT;
}

bool reconnectMQTT() {
  bool retval = true;
  Serial.print("--> reconnect MQTT..");
  if(!WiFi.isConnected()) WiFiConnect();
  //Serial.print("Attempting MQTT connection...");
  String clientId = "AirSender-";
  clientId += String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str(),MQTT_LOGIN,MQTT_PASSWD)) {
    Serial.print("connected...");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    retval = false;
  }
  //Serial.print("--> reconnect MQTT End : "); 
  Serial.println(retval ? "OK" : "FAILED");
  return retval;
}
 */
/////////////////////////////////////////////////////////////////////////
// ESP32 deep sleep & machine state 
/////////////////////////////////////////////////////////////////////////
#include <esp_sleep.h>

esp_sleep_wakeup_cause_t wakeup_reason;

void print_wakeup_reason(){
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    case ESP_SLEEP_WAKEUP_GPIO : Serial.println("Wakeup caused by GPIO"); break;
    case ESP_SLEEP_WAKEUP_ALL : Serial.println("Wakeup caused by ALL"); break;
    case ESP_SLEEP_WAKEUP_UART : Serial.println("Wakeup caused by UART"); break;
    case ESP_SLEEP_WAKEUP_UNDEFINED : Serial.println("Wakeup Undefined"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }

}

//////////////////////////////////////
// Timzone and daylight 
//////////////////////////////////////



//////////////////////////////////////
// OLED 
//////////////////////////////////////
#include <U8x8lib.h>
#include <U8g2lib.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

boolean flip_display = true;
uint8_t contrast_display = 1;
int contrast_up_down = 1;

U8X8_SSD1306_128X64_NONAME_HW_I2C display(/* reset= */ OLED_RST);
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C display();
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R2,OLED_RST);

void initOLED(void)
{ 
  display.begin();
  display.setFlipMode(flip_display);
  display.clearDisplay();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.noInverse();
  display.print("display:"); display.print(display.getCols());
  display.print("x"); display.println(display.getRows());
  display.display();
}

void displayWait() {
  display.setFlipMode(flip_display);
  time_t nowTime = timeClient.getEpochTime();
  tm *n = localtime(&nowTime);

  if((n->tm_hour > 21) || (n->tm_hour < 10)) {
    display.noDisplay();
    return;
  }
  
  const uint8_t CONTRAST_PAS = 4;
  if (contrast_display > 254 - CONTRAST_PAS) contrast_up_down = -CONTRAST_PAS;
  if (contrast_display < CONTRAST_PAS + 1) contrast_up_down = +CONTRAST_PAS;
  contrast_display += contrast_up_down;

  display.setContrast(contrast_display);
  display.setCursor(0,0);
  display.setFont(u8x8_font_chroma48medium8_r);  
  display.setInverseFont(true);
  display.inverse();
  display.printf("LoRa Sender     ");
  display.setInverseFont(false);
  display.setCursor(0,1); 
  display.printf("DHT  %4.1fC %.0f%%",DHTtemperature, DHThumidity);
  display.setCursor(0,2); 
  display.printf("Air  %4.0f (%4.0f)",correctedPPM,ppm); 
  display.setCursor(0,7);
  display.printf("%02d:%02d:%02d",n->tm_hour,n->tm_min,n->tm_sec);
  display.setCursor(14,7); WiFi.isConnected() ? display.print("Wi") : display.print("  ");
  display.setCursor(8, 7); LoRa.isTransmitting() ? display.print("Lo") : display.print("  ");
  //display.setCursor(11,7); mqttClient.connected() ? display.print("Mq") : display.print("  ");
  display.display();
}

//////////////////////////////////////
// SETUP
//////////////////////////////////////
void setup() {
  Serial.begin(115200);

  print_wakeup_reason();

  // OLED INIT
  Wire.begin(OLED_SDA, OLED_SCL);

  initOLED();

  // LORA INIT
  display.print("LoRa "); display.display();
  LoRa.setPins(LORA_CS,LORA_RST,LORA_IRQ);
  if (!LoRa.begin(LORA_BAND)) {
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setCodingRate4(LORA_CODINGRATE_DENOMINATOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setTxPower(LORA_TX_POWER);
    Serial.println("LoRa init failed. Check your connections.");
    display.println("LoRa init failed !");
    display.display();
    while (true);
  }
  display.println(" started"); display.display();
  //LoRa_rxMode();

  //attachInterrupt(digitalPinToInterrupt(LORA_IRQ), LoraIRQ, RISING);
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);

  // WiFi init
  display.print("WiFi "); display.display();
  WiFi.begin(AP_NAME, AP_PASSRHRASE);
  WiFi.onEvent(onWifiEvent);
  display.println("started"); display.display();

  // OTA
  display.print("OTA "); display.display();
  initOTA();
  display.println("started"); display.display();

  // NTPClient init
  display.print("NTP "); display.display();
  timeClient.begin();
  display.println(" started"); display.display();

  // DHT 
  display.print("DHT22");display.display();
  dht22.begin();
  display.println(" started");display.display();

  // send Data timer 
  timerSend = timerBegin(0, 80, true);
  timerAttachInterrupt(timerSend, &onSend, true);
  timerAlarmWrite(timerSend, time_to_send_msecs * mS_TO_S_FACTOR, true);

  // NTP timer
  timerNTP = timerBegin(1, 80, true);
  timerAttachInterrupt(timerNTP, &onNTP, true);
  timerAlarmWrite(timerNTP, time_to_update_NTP_secs * uS_TO_S_FACTOR, true);

  // Air sense timer
  timerAir = timerBegin(2, 80, true);
  timerAttachInterrupt(timerAir, &onAir, true);
  timerAlarmWrite(timerAir, time_to_update_Air_msecs * mS_TO_S_FACTOR, true);

  // MQTT local mosquitto
  /* mqttClient.setServer(MQTT_SERVER,MQTT_PORT);
  mqttClient.setCallback(receivedMQTT);
  display.println("MQTT started");
  display.display(); */

  delay(2000);

}

//////////////////////////////////////
// LOOP
//////////////////////////////////////

void loop(void)
{

  switch (state) {
  case INIT:
    timerAlarmEnable(timerSend);
    timerAlarmEnable(timerNTP);
    timerAlarmEnable(timerAir);
    WiFiConnect();
    /* reconnectMQTT();
    Serial.printf("MQTT_Local : subscribe to [%s] ",subStrTempInt);
    if (mqttClient.subscribe(subStrTempInt)) { Serial.println("OK"); } else { Serial.println("FAILED !"); } */
    MQ135_read();
    //timeClient.update();
    display.clearDisplay();
    state = NTP;
    break;
  
  case WAIT:
    //mqttClient.loop();
    break;

  case MQTT:
    //temperature = MQTTtemperature;
    state = WAIT;
    break;

  case NTP:
    if(!WiFi.isConnected()) WiFiConnect();
    timeClient.update();
    // WiFi.disconnect(true);
    state = WAIT;
    break;

  case AIR:
    MQ135_read();
    readDHT();
    temperature = isnan(DHTtemperature) ? temperature : DHTtemperature ;
    humidity = isnan(DHThumidity) ? humidity : DHThumidity;
    state = WAIT;
    break;

  case SEND:
    prepareTxFrame(1);
    LoRa_sendMessage();
    //flip_display = flip_display == 1 ? 0 : 1 ;
    state = WAIT;
    break;

  case SENT:
    display.setCursor(0,4);
    display.printf("sent %d bytes\n",textSend.length());
    display.printf("time %s\n",timeClient.getFormattedTime().c_str());
    display.display();
    LoRa.idle();
    state = WAIT;
    break;

  default:
    state = INIT;
    break;
  }

  displayWait();

}
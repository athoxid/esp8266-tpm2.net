/*
 TPM2.NET protocol handler
 Works with Jinx! LED software
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <FastLED.h>
#include <WiFiManager.h>
#include <Ticker.h>

#define TMP2NET_IN_PORT 65506
#define TMP2NET_OUT_PORT 65442

WiFiUDP udp;

const uint16_t width = 8;
const uint16_t height = 8;
const uint16_t numleds = width * height;

const uint16_t payload = numleds * 3;

const uint8_t packet_start_byte = 0x9c;
const uint8_t packet_type_data = 0xda;
const uint8_t packet_type_cmd = 0xc0;
const uint8_t packet_type_response = 0xaa;
const uint8_t packet_end_byte = 0x36;
const uint8_t packet_response_ack = 0xac;

// maximum udp esp8266 payload = 1460 -> 484 rgb leds per frame (22x22 matrix)
// then you have to use packet numbers and split the payload
const uint16_t expected_packet_size = payload + 7;

uint8_t data[payload];

#if defined(NO_WIFI_MANAGER)
const char *ssid = "";
const char *password = "";

IPAddress local_ip(192, 168, 1, 200);
IPAddress gateway_ip(192, 168, 1, 1);
IPAddress subnet_ip(255, 255, 255, 0);
#endif

// Param for different pixel layouts
// TODO: verify vertical order (if starting from top or bottom)
const bool kMatrixSerpentineLayout = true;

uint16_t XY(uint8_t x, uint8_t y) {
  uint16_t i;

  if (kMatrixSerpentineLayout == false) {
    i = (y * width) + x;
  } else {
    if (y & 0x01) {
      // Odd rows run backwards
      uint8_t reverseX = (width - 1) - x;
      i = (y * width) + reverseX;
    } else {
      // Even rows run forwards
      i = (y * width) + x;
    }
  }
  return i;
}

#define LED_PIN 3

CRGB leds[numleds];
Ticker ticker;

void tick() {
  int state = digitalRead(BUILTIN_LED);
  digitalWrite(BUILTIN_LED, !state);
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //entered config mode, make led toggle faster
  ticker.detach();
  ticker.attach(0.2, tick);
}

const CRGB ipalette[3] = {
  { 0, 0, 0}, // black
  { 255, 255, 255}, // white
  { 255, 0, 200 } // pink
};
  
 const uint8_t invader[8][8] PROGMEM = {
   {0, 1, 0, 1, 1, 0, 1, 0},
   {0, 0, 1, 1, 1, 1, 0, 0},
   {0, 1, 2, 1, 1, 2, 1, 0},
   {1, 1, 1, 1, 1, 1, 1, 1},
   {1, 0, 1, 1, 1, 1, 0, 1},
   {1, 0, 1, 0, 0, 1, 0, 1},
   {0, 0, 1, 0, 0, 1, 0, 0},
   {0, 1, 1, 0, 0, 1, 1, 0}
};

void displayInvader() {
  for (uint8_t y = 0; y < height; ++y) {
    for (uint8_t x = 0; x < width; ++x) {
      uint8_t idx = pgm_read_byte(&invader[y][x]);
      leds[XY(x, y)] = ipalette[idx];
    }
  }
  FastLED.show();
  // TODO: display fps on OLED
  FastLED.getFPS();
}

void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(BUILTIN_LED, OUTPUT);
  ticker.attach(0.6, tick);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, numleds);
  FastLED.countFPS();
  FastLED.setBrightness(128); // limit power usage for testing
  // make the display show something  
  displayInvader();

#if defined(NO_WIFI_MANAGER)
Serial.print("Connecting to ");
Serial.println(ssid);

WiFi.begin(ssid, password);
  // is is necessary ?
  WiFi.config(local_ip, gateway_ip, subnet_ip);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
#else
  WiFiManager wifiManager;
  // wifiManager.resetSettings();
  // wifiManager.setDebugOutput(false);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.autoConnect("ledmatrix", "ledmatrix");
#endif
  ticker.detach();
  digitalWrite(BUILTIN_LED, HIGH);
  
 
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  long rssi = WiFi.RSSI();
   // -120 to 0 DBm
   // int quality;
//   if(rssi <= -100)
//   quality = 0;
// else if(rssi >= -50)
//   quality = 100;
// else
//   quality = 2 * (rssi + 100);
// Serial.print("signal quality:");
// Serial.println(quality);

/*
Android code
private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -55;
public static final int RSSI_LEVELS = 5;

public static int calculateSignalLevel(int rssi, int numLevels) {
        if (rssi <= MIN_RSSI) {
            return 0;
        } else if (rssi >= MAX_RSSI) {
            return numLevels - 1;
        } else {
            float inputRange = (MAX_RSSI - MIN_RSSI);
            float outputRange = (numLevels - 1);
            return (int)((float)(rssi - MIN_RSSI) * outputRange / inputRange);
        }
    }
    */

Serial.print("signal strength: ");
Serial.print(rssi);
Serial.println(" dBm");
udp.begin(TMP2NET_IN_PORT);

}

void sendAck() {
  udp.beginPacket(udp.remoteIP(), TMP2NET_OUT_PORT);
  udp.write(&packet_response_ack, 1);
  udp.endPacket();
}

void fillMatrix() {
  for (uint8_t y = 0; y < height; ++y) {
    uint8_t *ptr = data + y * 3 * width;
    for (uint8_t x = 0; x < width; ++x, ptr += 3) {
      leds[XY(x, y)].setRGB(ptr[0], ptr[1], ptr[2]);
    }
  }
  FastLED.show();
}

void fillMatrix(CRGB color) {
  // do not care about XY order
  for (uint16_t l = 0; l < numleds; ++l) {
      leds[l].setRGB(color.r, color.g, color.b);
  }
  
//  fill_solid(leds, numleds, color);
  FastLED.show();
}

uint32_t frameCounter = 0;

void loop() {
  uint16_t packet_size = udp.parsePacket();
     // then parse to check
  if (udp.read() != packet_start_byte)
    return;
  // packet type
  uint8_t ptype = udp.read();
  uint16_t frame_size = (udp.read() << 8) | udp.read();
  // skip packet number and number of packets
  udp.read();
  udp.read();

  switch (ptype) {
    case packet_type_response:
      sendAck();
      break;

    case packet_type_data: {
      if (frame_size != payload)
        return;
      // TODO: check if Jinx! sends data in snake order
      // then code will be much simpler and faster
      // TODO: fill the matrix directly, in the (char*)leds array
      // udp.read((char*)leds, payload);
      // FastLed.show();
      udp.read(data, payload);
      // now process the data (fill the matrix)
      fillMatrix();
      ++frameCounter;
      break;
    }
    case packet_type_cmd: {
      // read the command
      uint8_t command;
      uint8_t rgb[3];
      udp.read(&command, 1); // not used yet
      udp.read(rgb, 3);
      fillMatrix(CRGB(rgb[0], rgb[1], rgb[2]));
      break;
    }
  }
  // skip end byte, maybe not neccessary
  udp.read();
}

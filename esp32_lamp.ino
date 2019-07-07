// Copyright 2017 Mike Stunes.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "secret.h"

const char *LIGHT_STATE = "http://" BRIDGE "/api/" API_USERNAME "/lights/%s";
const char *LIGHT_PAYLOAD = "{\"on\": %s, \"bri\": %d, \"hue\": %d}";

const char STR_TRUE = "true"
const char STR_FALSE = "false"

const unsigned int BOUNCE_DELAY_MS = 500; // ms

typedef struct l_state {
  bool ison;
  int bri;
  int hue;
} l_state_t;

l_state_t lights[LIGHTS_COUNT];
int light_ids[LIGHTS_COUNT];
int input_pins[LIGHTS_COUNT];
int current_pin_idx = -1;

char msgbuff[50];

unsigned long lastInterrupt;  // last interrupt time
volatile int shouldTrigger = 0;

const unsigned int CONNECT_TIMEOUT_MS = 30000;  // WiFi connnection timeout (ms)
const unsigned int WIFI_CHECK_MS = 30000;       // WiFi status check interval (ms)
unsigned long nextWifiCheck;  // next time to check WiFi status

// connectToWiFi adapted from ESP32 example code. See, e.g.:
// https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiClient/WiFiClient.ino
void connectToWiFi() {
  unsigned long startTime = millis();
  Serial.println("Connecting to: " + String(SSID));

  WiFi.disconnect();
  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - startTime > CONNECT_TIMEOUT_MS) {
      Serial.println();
      Serial.println("Failed to connect.");
      return;
    }
  }

  Serial.println();
  Serial.println("Connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);

  nextWifiCheck = millis() + WIFI_CHECK_MS;
}

void putJson(const char *url, char *content) {
  Serial.println("putJson");
  Serial.printf("PUT %s: %s\n", url, content);

  HTTPClient http;
  http.begin(url);
  int httpCode = http.PUT(content);
  if (httpCode > 0) {
    Serial.printf("Code: %d\n", httpCode);
  } else {
    Serial.printf("Error: %s\n", http.errorToString(httpCode).c_str());
  }
  Serial.println(http.getString());
  http.end();
}

String getUrl(const char *url) {
  Serial.println("get");
  Serial.printf("GET %s\n", url);

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.printf("Code: %d\n", httpCode);
  } else {
    Serial.printf("Error: %s\n", http.errorToString(httpCode).c_str());
  }

  return http.getString();
}

char* getOnDataFor(int index) {
  char jsonbuff = malloc(128);
  sprintf(jsonbuff, LIGHT_PAYLOAD, lights[index].ison ? STR_TRUE : STR_FALSE, lights[index].bri, lights[index].hue);
  Serial.printf("Creating json: %s\n", jsonbuff); 
  return jsonbuff;
}

void toggleLight(int index) {
  sprintf(msgbuff, "Turning light %d %s", light_ids[index], lights[index].ison ? "off" : "on");
  Serial.println(msgbuf);
  char urlbuff[100];
  sprintf(urlbuff, LIGHT_STATE, id);
  char *on_data = getOnDataFor(index);
  putJson(urlbuff, on_data);
  free(on_data);  
}

void handleButton(int index) {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterrupt) > BOUNCE_DELAY_MS) {
    Serial.println("Handling button event");
    lastInterrupt = currentTime;
    shouldTrigger = index;
  }
}

void handleButton0() { handleButton(0);

void getLightsState(int id) {
  char light_state_url[100];
  sprintf(light_state_url, LIGHT_STATE, id);
  String json_body = getUrl(light_state_url);
  StaticJsonBuffer<1024> json_buffer;
  JsonObject &root = json_buffer.parseObject(jsonBody);
  lights[id].on = root["state"]["on"];
  lights[id].bri = root["state"]["bri"];
  lights[id].hue = root["state"]["hue"];
}

void readLightsState() {
  for (int i=0; i < LIGHTS_COUNT; ++i) {
    getLightState(light_ids[i]);
  }
}

void populateArray(int *array, char *config_str) {
  char *mutable_str = calloc(strlen(config_str) + 1);
  char delim = ",";

  strcpy(mutable_str, config_str);

  char *ptr = strtok(mutableStr, delim);
  int i = 0;

  while(ptr != NULL && i < LIGHTS_COUNT) {
    *(array++) = atoi(ptr);
    ptr = strtok(NULL, delim);
  }

  free(mutable_str);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  lights = calloc(sizeof(l_state_t) * LIGHTS_COUNT);
  populateArray(light_ids, LIGHT_IDS);
  populateArray(input_pins, BUTTON_PINS);

  pinMode(LED_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT_PULLUP);

  connectToWiFi();

  for (int i=0; i < LIGHTS_COUNT)
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), handleButton, FALLING);
  Serial.println("Button interrupt enabled");
}

void loop() {
  if (shouldTrigger != -1) {
    readLightsState();
    toggleLight(light_pins[shouldTrigger]);
    shouldTrigger = -1;
  }

  // Check WiFi status and reconnect if necessary
  // https://www.reddit.com/r/esp32/comments/7trl0f/reconnect_to_wifi/dtfbfct/
  unsigned long currentTime = millis();
  if (currentTime > nextWifiCheck) {
    Serial.println("Checking WiFi status");
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected; will try reconnecting");
      connectToWiFi();
      // connectToWiFi will reset nextWifiCheck for us in this case
    } else {
      Serial.println("WiFi connected; will do nothing");
      nextWifiCheck = currentTime + WIFI_CHECK_MS;
    }
  }
}

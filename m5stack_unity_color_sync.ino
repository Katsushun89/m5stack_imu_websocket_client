#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <M5Stack.h>
#include <map>
#include "config.h"

WebSocketsClient webSocket;
DynamicJsonDocument doc(1024);

std::map<std::string, uint32_t> colorMap{
    {"red", RED},
    {"green", GREEN},
    {"blue", BLUE}
};

std::string parseReceivedJson(uint8_t *payload)
{
  char *json = (char *)payload;
  DeserializationError error = deserializeJson(doc, json);
  
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return "none";
  }

  JsonObject obj = doc.as<JsonObject>();

  // You can use a String to get an element of a JsonObject
  // No duplication is done.
  return obj[String("color")];
}

void syncColor(uint8_t *payload)
{
  std::string color = parseReceivedJson(payload);

  Serial.printf("color: %s\n", color.c_str());
  //M5.Lcd.fillRect(60, 20, 200, 200, colorMap[color]);
  M5.Lcd.fillScreen(colorMap[color]);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			Serial.printf("[WSc] Disconnected!\n");
			break;
		case WStype_CONNECTED:
			Serial.printf("[WSc] Connected to url: %s\n", payload);
			//webSocket.sendTXT("Connected");
			break;
		case WStype_TEXT:
			Serial.printf("[WSc] get text: %s\n", payload);
      syncColor(payload);
			break;
		case WStype_BIN:
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
	}
}

void setupWiFi()
{
  WiFi.begin(ssid, passwd);

  // Wait some time to connect to wifi
  for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
      Serial.print(".");
      delay(1000);
  }

  // Check if connected to wifi
  if(WiFi.status() != WL_CONNECTED) {
      Serial.println("No Wifi!");
      return;
  }

  Serial.println("Connected to Wifi, Connecting to server.");
	// server address, port and URL
	webSocket.begin("192.168.10.11", 8080, "/");

	// event handler
	webSocket.onEvent(webSocketEvent);

	// use HTTP Basic Authorization this is optional remove if not needed
	//webSocket.setAuthorization("user", "Password");

	// try ever 5000 again if connection has failed
	webSocket.setReconnectInterval(5000);
}

void setup()
{
  Serial.begin(115200);
  // Power ON Stabilizing...
  delay(500);
  M5.begin();

  setupWiFi();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.setTextSize(2);
}

void loop() {
  bool isPressedBtnA = false;
  bool isPressedBtnB = false;
  bool isPressedBtnC = false;
  if(M5.BtnA.wasPressed() || M5.BtnA.isPressed())
  {
    isPressedBtnA = true;
  }
  if(M5.BtnB.wasPressed() || M5.BtnB.isPressed())
  {
    isPressedBtnB = true;
  }
  if(M5.BtnC.wasPressed() || M5.BtnC.isPressed())
  {
    isPressedBtnC = true;
  }

  static uint32_t pre_send_time = 0;
  uint32_t time = millis();
  if(time - pre_send_time > 100){
    pre_send_time = time;
    String isPressedBtnAStr = (isPressedBtnA ? "true": "false");
    String isPressedBtnBStr = (isPressedBtnB ? "true": "false");
    String isPressedBtnCStr = (isPressedBtnC ? "true": "false");
    String btn_str = "{\"red\":" + isPressedBtnAStr + 
      ", \"green\":" + isPressedBtnBStr + 
      ", \"blue\":" + isPressedBtnCStr + "}";
    //Serial.println(btn_str);
    webSocket.sendTXT(btn_str);
  }
  webSocket.loop();

  M5.update();
}

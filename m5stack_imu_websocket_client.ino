#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <M5Stack.h>
//#include "utility/MPU9250.h"
#include "config.h"
#include "imu.h"

// Devices
//MPU9250 IMU;
using namespace websockets;

WebsocketsClient client;

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
    // try to connect to Websockets server
    bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
    if(connected) {
        Serial.println("Connected!");
        client.send("Hello Server");
    } else {
        Serial.println("Not Connected!");
    }
    
    // run callback when messages are received
    client.onMessage([&](WebsocketsMessage message){
        Serial.print("Got Message: ");
        Serial.println(message.data());
    });

}

void setup()
{
  Serial.begin(115200);
  // Power ON Stabilizing...
  delay(500);
  M5.begin();
  Wire.begin();

//  initIMU(&IMU);

  setupWiFi();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.setTextSize(2);
}

void loop() {
#if 0
  if(calcIMU(&IMU)){
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(IMU.yaw, 2);
        Serial.print(", ");
        Serial.print(IMU.pitch, 2);
        Serial.print(", ");
        Serial.print(IMU.roll, 2);
      }
      if(M5.BtnC.isPressed()){
        sendUdp(&IMU);
      }else{
        Serial.println("");
      }
  }
#endif
    if(client.available()) {
        client.poll();
    }
  delay(100);
  M5.update();
}

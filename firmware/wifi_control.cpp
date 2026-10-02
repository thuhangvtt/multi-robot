#include "wifi_control.h"
#include "config.h"
#include "imu.h"
#include "motor.h"
#include "robot_control.h"
#include "servo_control.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {
WebServer server(80);
bool serverReady = false;

void sendJsonStatus() {
  const String ip = WIFI_USE_ACCESS_POINT ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String body = "{\"robotId\":" + String(ROBOT_ID);
  body += ",\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "connected" : "offline") + "\"";
  body += ",\"ip\":\"" + ip + "\"";
  body += ",\"left\":" + String(motorLeftOutput());
  body += ",\"right\":" + String(motorRightOutput());
  body += ",\"yaw\":" + String(imuYawDegrees(), 2);
  body += ",\"imuReady\":" + String(imuIsReady() ? "true" : "false") + "}";
  server.send(200, "application/json", body);
}

void handleControl() {
  const String type = server.arg("dtype");
  if (type == "speed") {
    const int left = server.hasArg("left") ? server.arg("left").toInt() : server.arg("servo1").toInt();
    const int right = server.hasArg("right") ? server.arg("right").toInt() : server.arg("servo2").toInt();
    robotSetMotion(left, right);
    server.send(200, "text/plain", "OK SPEED");
  } else if (type == "stop") {
    robotStop();
    server.send(200, "text/plain", "OK STOP");
  } else if (type == "straight") {
    robotSetStraight(server.arg("enabled").toInt() != 0);
    server.send(200, "text/plain", "OK STRAIGHT");
  } else if (type == "turn") {
    const String direction = server.arg("direction");
    if (direction == "left") robotTurn(-1);
    else if (direction == "right") robotTurn(1);
    else { server.send(400, "text/plain", "ERROR direction must be left or right"); return; }
    server.send(200, "text/plain", "OK TURN");
  } else if (type == "servo") {
    servoSetAngles(server.arg("a").toInt(), server.arg("b").toInt());
    server.send(200, "text/plain", "OK SERVO");
  } else if (type == "zeroYaw") {
    imuResetYaw();
    server.send(200, "text/plain", "OK ZEROYAW");
  } else {
    server.send(400, "text/plain", "ERROR unsupported dtype");
  }
}

void handleRoot() {
  server.send(200, "text/plain", "robot firmware; use /control and /status");
}
}

void wifiControlInit() {
  char hostname[24];
  snprintf(hostname, sizeof(hostname), "robot-%u", ROBOT_ID);
  if (WIFI_USE_ACCESS_POINT) {
    char apSsid[24];
    snprintf(apSsid, sizeof(apSsid), "Robot-%u", ROBOT_ID);
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(apSsid, WIFI_AP_PASSWORD)) {
      Serial.println("WIFI ERROR: access point start failed");
      return;
    }
    Serial.print("WIFI AP READY ssid=");
    Serial.print(apSsid);
    Serial.print(" ip=");
    Serial.println(WiFi.softAPIP());
  } else {
    if (String(WIFI_SSID) == "YOUR_WIFI_NAME") {
      Serial.println("WIFI DISABLED: set WIFI_SSID in config.h");
      return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WIFI CONNECTING");
    const uint32_t deadline = millis() + 10000;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WIFI ERROR: connection timeout");
      return;
    }
    Serial.print("WIFI READY robot=");
    Serial.print(ROBOT_ID);
    Serial.print(" ip=");
    Serial.println(WiFi.localIP());
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/status", HTTP_GET, sendJsonStatus);
  server.begin();
  serverReady = true;
}

void wifiControlTask() {
  if (serverReady) server.handleClient();
}
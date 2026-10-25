/**
 * @file wifi_control.cpp
 * @brief Triển khai HTTP WebServer nhận lệnh từ Server trung tâm.
 */

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
uint32_t lastWifiCheck = 0;

int extractJsonInt(const String& str, const char* key, int defaultValue) {
  int keyIdx = str.indexOf(key);
  if (keyIdx < 0) return defaultValue;
  int colonIdx = str.indexOf(':', keyIdx);
  if (colonIdx < 0) return defaultValue;

  int start = colonIdx + 1;
  while (start < (int)str.length() && (str[start] == ' ' || str[start] == '\t')) {
    start++;
  }
  int end = start;
  if (end < (int)str.length() && (str[end] == '-' || str[end] == '+')) {
    end++;
  }
  while (end < (int)str.length() && isDigit(str[end])) {
    end++;
  }
  if (end > start) {
    return str.substring(start, end).toInt();
  }
  return defaultValue;
}

void handleRoot() {
  server.send(200, "text/plain", "PARROT Robot OK");
}

void handleCommandPost() {
  String body = server.arg("plain");

  int left = extractJsonInt(body, "\"l\"", 90);
  int right = extractJsonInt(body, "\"r\"", 90);
  int action = extractJsonInt(body, "\"action\"", 0);
  if (action == 0) {
    action = extractJsonInt(body, "\"e\"", 0);
  }

  if (server.hasArg("l")) left = server.arg("l").toInt();
  if (server.hasArg("r")) right = server.arg("r").toInt();
  if (server.hasArg("action")) action = server.arg("action").toInt();
  else if (server.hasArg("e")) action = server.arg("e").toInt();

  robotApplyCommand(left, right, action);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleStop() {
  robotStop();
  server.send(200, "text/plain", "STOPPED");
}

void sendJsonStatus() {
  const String ip = WIFI_USE_ACCESS_POINT ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String body = "{";
  body += "\"robotId\":" + String(ROBOT_ID) + ",";
  body += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "connected" : "offline") + "\",";
  body += "\"ip\":\"" + ip + "\",";
  body += "\"motorLeft\":" + String(motorGetLeftSpeed()) + ",";
  body += "\"motorRight\":" + String(motorGetRightSpeed()) + ",";
  body += "\"lifted\":" + String(servoIsLifted() ? 1 : 0) + ",";
  body += "\"angleA\":" + String(servoGetAngleA()) + ",";
  body += "\"angleB\":" + String(servoGetAngleB()) + ",";
  body += "\"fault\":" + String(drv8833IsFault() ? 1 : 0) + ",";
  body += "\"yaw\":" + String(imuYawDegrees(), 2) + ",";
  body += "\"timedOut\":" + String(robotIsTimedOut() ? "true" : "false");
  body += "}";
  server.send(200, "application/json", body);
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
      Serial.println("[Wi-Fi] ERROR: Không thể khởi tạo Access Point!");
      robotSetLedRed(true);
      return;
    }
    Serial.print("[Wi-Fi] AP Sẵn sàng: ");
    Serial.print(apSsid);
    Serial.print(" - IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname);

    if (USE_STATIC_IP) {
      if (!WiFi.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, DNS_IP)) {
        Serial.println("[Wi-Fi] Cảnh báo: Cấu hình Static IP thất bại, chuyển về DHCP");
      }
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[Wi-Fi] Đang kết nối tới '");
    Serial.print(WIFI_SSID);
    Serial.print("'");

    uint32_t deadline = millis() + 10000;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
      delay(300);
      Serial.print(".");
      digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[Wi-Fi] THẤT BẠI: Quá thời gian kết nối!");
      robotSetLedRed(true);
      return;
    }

    robotSetLedRed(false);
    robotSetLedBlue(true);

    Serial.print("[Wi-Fi] KẾT NỐI THÀNH CÔNG! Robot ID: ");
    Serial.print(ROBOT_ID);
    Serial.print(" | IP: ");
    Serial.println(WiFi.localIP());
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_POST, handleCommandPost);
  server.on("/cmd", HTTP_GET, handleCommandPost);
  server.on("/status", HTTP_GET, sendJsonStatus);
  server.on("/stop", HTTP_GET, handleStop);

  server.begin();
  serverReady = true;
  Serial.println("[HTTP Server] Sẵn sàng lắng nghe trên cổng 80 tại endpoint /cmd");
}

void wifiControlTask() {
  if (serverReady) {
    server.handleClient();
  }

  const uint32_t now = millis();
  if (!WIFI_USE_ACCESS_POINT && now - lastWifiCheck > 3000) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      robotSetLedRed(true);
      robotSetLedBlue(false);
      WiFi.reconnect();
    } else {
      robotSetLedRed(robotIsTimedOut());
    }
  }
}
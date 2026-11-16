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

bool readCommandInt(const char* key, int& value) {
  if (!server.hasArg(key)) return false;
  String text = server.arg(key);
  if (!text.length() || text.length()>11) return false;
  bool negative = text[0]=='-';
  unsigned start = negative ? 1 : 0;
  if (start == text.length()) return false;
  int64_t number=0;
  for (unsigned i=start; i<text.length(); ++i) {
    if (text[i]<'0' || text[i]>'9') return false;
    number=number*10+(text[i]-'0');
    if (number>2147483647LL) return false;
  }
  value=(int)(negative ? -number : number);
  return true;
}

void handleRoot() { server.send(200, "text/plain", "PARROT Robot OK"); }

void handleCommandPost() {
  int left,right,action=0,angleA=0,angleB=0,stabilizer=0;
  bool angles=server.hasArg("servo_a") || server.hasArg("servo_b");
  if (!readCommandInt("l",left) || !readCommandInt("r",right) ||
      left<50 || left>130 || right<50 || right>130 ||
      (server.hasArg("action") && (!readCommandInt("action",action) || action<0 || action>2)) ||
      (angles && (!readCommandInt("servo_a",angleA) || !readCommandInt("servo_b",angleB) ||
                  angleA<0 || angleA>180 || angleB<0 || angleB>180)) ||
      (server.hasArg("stabilizer") && (!readCommandInt("stabilizer",stabilizer) || stabilizer<0 || stabilizer>1))) {
    server.send(400,"text/plain","Expected l/r 50..130, action 0..2, optional servo angles 0..180"); return;
  }
  if (drv8833IsFault()) { motorStop(); server.send(503,"text/plain","Motor driver fault"); return; }
  robotApplyCommand(left,right,action);
  if (angles) servoSetAngles(angleA,angleB);
  if (server.arg("zero_yaw")=="1") imuResetYaw();
  if (server.hasArg("stabilizer")) motorSetStabilizer(stabilizer==1);
  server.send(200,"text/plain","OK");
}

void handleParrotCommand() {
  String dtype=server.arg("dtype");
  int left,right,power;
  if (dtype=="speed" && readCommandInt("servo1",left) && readCommandInt("servo2",right) &&
      left>=50 && left<=130 && right>=50 && right<=130) {
    if (drv8833IsFault()) { motorStop(); server.send(503,"text/plain","Motor driver fault"); return; }
    robotApplyCommand(left,right,0);
  } else if (dtype=="pallet" && readCommandInt("power",power) && (power==0 || power==1)) {
    servoSetPalletAction(power==1 ? 1 : 2);
  } else { server.send(400,"text/plain","Invalid PARROT speed/pallet command"); return; }
  server.send(200,"text/plain","OK");
}

void handleStop() { robotStop(); server.send(200,"text/plain","STOPPED"); }

void sendJsonStatus() {
  const String ip = WIFI_USE_ACCESS_POINT ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String body = "{";
  body += "\"uptime_ms\":" + String(millis()) + ",";
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
  server.on("/", HTTP_POST, handleParrotCommand);
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

#pragma once
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

class SerialLine {
  char text[96] = {};
  size_t used = 0;
  bool overflow = false;

public:
  void poll(void (*handler)(const char *)) {
    for (int i = 0; i < 96 && Serial.available(); ++i) {
      char ch = Serial.read();
      if (ch == '\r' || ch == '\n') {
        if (overflow)
          Serial.println("ERROR line too long; discarded");
        else if (used) {
          text[used] = 0;
          handler(text);
        }
        used = 0;
        overflow = false;
      } else if (!overflow) {
        if (used < sizeof(text) - 1)
          text[used++] = ch;
        else
          overflow = true;
      }
    }
  }
};

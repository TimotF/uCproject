#include <Arduino.h>
#include "pin.h"

void setup()
{
  // CNY70 pin init
  pinMode(PIN_EN_3V3, OUTPUT);
  digitalWrite(PIN_EN_3V3, LOW);

  // ENDSTOP pin init
  pinMode(PIN_ENDSTOP_1, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_2, INPUT_PULLUP);

  // MOTOR pin init
  pinMode(PIN_MOT_SLEEP, OUTPUT);
  digitalWrite(PIN_MOT_SLEEP, LOW);
  pinMode(PIN_MOT_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_STEER1, OUTPUT);
  pinMode(PIN_MOT_STEER2, OUTPUT);
  pinMode(PIN_MOT_PROP1, OUTPUT);
  pinMode(PIN_MOT_PROP2, OUTPUT);

  // OTHER pin init
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

void loop()
{
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  delay(500);
}
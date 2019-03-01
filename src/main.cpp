#include <Arduino.h>
#include "pin.h"

#define BACKGROUND_TASK_PRIORITY 0
#define BACKGROUND_TASK_CORE 0

/**
 * This task is used to run all background process
 */
void backgroundTask(void *pvParameters)
{
  while (1)
  {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(500);
  }
}

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

  // RTOS init

  xTaskCreatePinnedToCore(
      backgroundTask,           /* Function to implement the task */
      "coreTask",               /* Name of the task */
      10000,                    /* Stack size in words */
      NULL,                     /* Task input parameter */
      BACKGROUND_TASK_PRIORITY, /* Priority of the task */
      NULL,                     /* Task handle. */
      BACKGROUND_TASK_CORE);    /* Core where the task should run */
}

void loop()
{
}
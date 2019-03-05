#include <Arduino.h>
#include "pin.h"
#include <Wire.h>

#define LOG(f_, ...)                    \
  {                                     \
    Serial.printf("[%ld] ", millis());  \
    Serial.printf((f_), ##__VA_ARGS__); \
  }

#define BACKGROUND_TASK_PRIORITY 0
#define BACKGROUND_TASK_CORE 0
#define SENSORS_TASK_PRIORITY 3
#define SENSORS_TASK_CORE 0

#define CH_MOT_PROP1 2
#define CH_MOT_PROP2 3
#define PWM_FREQUENCY 500
#define PWM_RESOLUTION 8

int s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0, s7 = 0, s8 = 0, s9 = 0, s10 = 0, endstop = 0;

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

/**
 * This task is used to refresh all sensors
 */
void sensorsTask(void *pvParameters)
{
  while (1)
  {
    delay(10);
    s1 = analogRead(PIN_CNY70_1);
    s2 = analogRead(PIN_CNY70_2);
    s3 = analogRead(PIN_CNY70_3);
    s4 = analogRead(PIN_CNY70_4);
    s5 = analogRead(PIN_CNY70_5);
    s6 = analogRead(PIN_CNY70_6);
    s7 = analogRead(PIN_CNY70_7);
    s8 = analogRead(PIN_CNY70_8);
    s9 = analogRead(PIN_CNY70_9);
    s10 = analogRead(PIN_CNY70_10);
    endstop = digitalRead(PIN_ENDSTOP_1);

    LOG("Sensors : s1=%d,\ts2=%d,\ts3=%d,\ts4=%d,\ts5=%d,\ts6=%d,\ts7=%d,\ts8=%d,\ts9=%d,\ts10=%d,\tendstop=%d\n", s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, endstop);
  }
}

void setup()
{
  // Serial port init (debug purposes)
  Serial.begin(115200);

  delay(500);

  // I2C init
  Wire.begin();

  for (uint8_t address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  // CNY70 pin init
  pinMode(PIN_EN_3V3, OUTPUT);
  digitalWrite(PIN_EN_3V3, LOW);

  // ENDSTOP pin init
  pinMode(PIN_ENDSTOP_1, INPUT_PULLUP);

  // MOTOR pin init
  pinMode(PIN_MOT_SLEEP, OUTPUT);
  digitalWrite(PIN_MOT_SLEEP, LOW);
  pinMode(PIN_MOT_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_STEER1, OUTPUT);
  pinMode(PIN_MOT_STEER2, OUTPUT);
  pinMode(PIN_MOT_PROP1, OUTPUT);
  pinMode(PIN_MOT_PROP2, OUTPUT);
  ledcSetup(CH_MOT_PROP1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(CH_MOT_PROP2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOT_PROP1, CH_MOT_PROP1);
  ledcAttachPin(PIN_MOT_PROP2, CH_MOT_PROP2);

  // OTHER pin init
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // RTOS init

  xTaskCreatePinnedToCore(
      backgroundTask,           /* Function to implement the task */
      "backgroundTask",         /* Name of the task */
      10000,                    /* Stack size in words */
      NULL,                     /* Task input parameter */
      BACKGROUND_TASK_PRIORITY, /* Priority of the task */
      NULL,                     /* Task handle. */
      BACKGROUND_TASK_CORE);    /* Core where the task should run */

  xTaskCreatePinnedToCore(
      sensorsTask,           /* Function to implement the task */
      "sensorsTask",         /* Name of the task */
      10000,                 /* Stack size in words */
      NULL,                  /* Task input parameter */
      SENSORS_TASK_PRIORITY, /* Priority of the task */
      NULL,                  /* Task handle. */
      SENSORS_TASK_CORE);    /* Core where the task should run */
}

void left()
{
  digitalWrite(PIN_MOT_STEER1, HIGH);
  digitalWrite(PIN_MOT_STEER2, LOW);
}

void right()
{
  digitalWrite(PIN_MOT_STEER1, LOW);
  digitalWrite(PIN_MOT_STEER2, HIGH);
}

void straight()
{
  digitalWrite(PIN_MOT_STEER1, LOW);
  digitalWrite(PIN_MOT_STEER2, LOW);
}

void forward(byte speed)
{
  ledcWrite(CH_MOT_PROP1, speed);
  ledcWrite(CH_MOT_PROP2, 0);
}

void backward(byte speed)
{
  ledcWrite(CH_MOT_PROP1, 0);
  ledcWrite(CH_MOT_PROP2, speed);
}

void stop()
{
  ledcWrite(CH_MOT_PROP1, 0);
  ledcWrite(CH_MOT_PROP2, 0);
}

void disableSensors()
{
  digitalWrite(PIN_EN_3V3, LOW);
}

void enableSensors()
{
  digitalWrite(PIN_EN_3V3, HIGH);
}

void enableMotors()
{
  digitalWrite(PIN_MOT_SLEEP, HIGH);
}

void disableMotors()
{
  digitalWrite(PIN_MOT_SLEEP, LOW);
}

void loop()
{
  enableSensors();
  bool but = false;
  while (!but)
  {
    but = !digitalRead(PIN_BUTTON);
    delay(1);
  }

  enableMotors();

  forward(128);
  straight();
  delay(1000);

  backward(128);
  right();
  delay(1000);

  forward(128);
  straight();
  delay(1000);

  backward(128);
  left();
  delay(1000);

  straight();
  stop();

  delay(1000);
  disableMotors();
}
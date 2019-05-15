#include <Arduino.h>
#include "pin.h"
#include <Wire.h>

#include "I2Cdev.h"
#include "HMC5883L.h"

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

#define BRAKE_SPEED 220
#define BRAKE_DELAY 100
#define FORWARD_TRAVEL_SPEED 70
#define FORWARD_LEFT_TRAVEL_SPEED 70
#define BACKWARD_TRAVEL_SPEED 90
#define EVENT_DELAY 700
#define FORWARD_DELAY 2000
#define LAST_EVENT_DELAY 3000

enum Direction
{
  FORWARD,
  FORWARD_LEFT,
  FORWARD_RIGHT,
  BACKWARD,
  BACKWARD_LEFT,
  BACKWARD_RIGHT,
  CORNER_RIGHT,
  CORNER_LEFT
};

enum Event
{
  LINE_FRONT_LEFT,
  LINE_FRONT_RIGHT,
  LINE_REAR_LEFT,
  LINE_REAR_RIGHT,
  WALL_FRONT_LEFT,
  WALL_FRONT_RIGHT,
  WALL_SIDE_LEFT,
  WALL_SIDE_RIGHT,
  NONE
};

Direction dir = FORWARD;
Direction cornerDir = BACKWARD;
Event event = NONE;
Event lastEvent = NONE;
uint32_t timerEvent = 0;
uint32_t timerLastEvent = 0;

int sLFL = 0,      // sensor Left Front Left
    sLFR = 0,      // sensor Left Front Right
    sL = 0,        // sensor Left
    sLR = 0,       // sensor Left Rear
    sRFL = 0,      // sensor Right Front Left
    sRFR = 0,      // sensor Right Front Right
    sR = 0,        // sensor Right
    sRR = 0,       // sensor Right Rear
    endstopLF = 0, // endstop Left Front
    endstopRF = 0, // endstop Right Front
    endstopLS = 0, // endstop Left Side
    endstopRS = 0; // endstop Right Side

HMC5883L mag;

int16_t mx, my, mz;

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
    sLFL = analogRead(PIN_CNY70_2);
    sLFR = analogRead(PIN_CNY70_1);
    sL = analogRead(PIN_CNY70_3);
    sLR = analogRead(PIN_CNY70_4);
    sRFL = analogRead(PIN_CNY70_7);
    sRFR = analogRead(PIN_CNY70_6);
    sR = analogRead(PIN_CNY70_8);
    sRR = analogRead(PIN_CNY70_10);
    endstopLF = !digitalRead(PIN_ENDSTOP_FRONT_LEFT);
    endstopRF = !digitalRead(PIN_ENDSTOP_FRONT_RIGHT);
    endstopLS = !digitalRead(PIN_ENDSTOP_SIDE_LEFT);
    endstopRS = !digitalRead(PIN_ENDSTOP_SIDE_RIGHT);

    // Events. Less prioritary first
    if (endstopLF)
      event = WALL_FRONT_LEFT;

    if (endstopRF)
      event = WALL_FRONT_RIGHT;

    if (endstopLS)
      event = WALL_SIDE_LEFT;

    if (endstopRS)
      event = WALL_SIDE_RIGHT;

    if (sLR > 4000)
      event = LINE_REAR_LEFT;

    if (sRR > 4000)
      event = LINE_REAR_RIGHT;

    if (sLFL > 4000 || sLFR > 4000 || sL > 4000)
      event = LINE_FRONT_LEFT;

    if (sRFL > 4000 || sRFR > 4000 || sR > 4000)
      event = LINE_FRONT_RIGHT;

      // LOG("Sensors : \nsLFL = \t%d\nsLFR = \t%d\nsL = \t%d\nsLR = \t%d\nsRFL = \t%d\nsRFR = \t%d\nsR = \t%d\nsRR = \t%d\nendstopLF = %d\nendstopRF = %d\nendstopLS = %d\nendstopRS = %d\n", sLFL, sLFR, sL, sLR, sRFL, sRFR, sR, sRR, endstopLF, endstopRF, endstopLS, endstopRS);
#if 0
   // read raw heading measurements from device
    mag.getHeading(&mx, &my, &mz);

    // display tab-separated gyro x/y/z values
    Serial.print("mag:\t");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\t");

    // To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if (heading < 0)
      heading += 2 * M_PI;
    Serial.print("heading:\t");
    Serial.println(heading * 180 / M_PI);
#endif
  }
}

void setup()
{
  // Serial port init (debug purposes)
  Serial.begin(115200);

  delay(500);

  // I2C init
  Wire.begin();
  mag.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  // CNY70 pin init
  pinMode(PIN_EN_3V3, OUTPUT);
  digitalWrite(PIN_EN_3V3, LOW);

  // ENDSTOP pin init
  pinMode(PIN_ENDSTOP_FRONT_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_SIDE_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_FRONT_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_SIDE_RIGHT, INPUT_PULLUP);

  // MOTOR pin init
  pinMode(PIN_MOT_SLEEP, OUTPUT);
  digitalWrite(PIN_MOT_SLEEP, LOW);
  pinMode(PIN_MOT_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_STEER1, OUTPUT);
  pinMode(PIN_MOT_STEER2, OUTPUT);
  pinMode(PIN_MOT_PROP1, OUTPUT);
  pinMode(PIN_MOT_PROP2, OUTPUT);
  digitalWrite(PIN_MOT_SLEEP, LOW);
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
  bool but = false;
  enableSensors();
  while (!but)
  {
    but = !digitalRead(PIN_BUTTON);
    delay(1);
  }
  enableMotors();
  while (1)
  {

    if (millis() - timerLastEvent > LAST_EVENT_DELAY)
    {
      lastEvent = NONE;
      timerLastEvent = millis();
    }

    switch (dir)
    {
    case FORWARD:
    {
      straight();
      if (millis() - timerEvent < 100)
        forward(150);
      else
        forward(FORWARD_TRAVEL_SPEED);

      if (millis() - timerEvent > FORWARD_DELAY)
      {
        dir = FORWARD_LEFT;
        timerEvent = millis();
        event = NONE;
        break;
      }

      switch (event)
      {
      case LINE_FRONT_LEFT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_LEFT;
        timerEvent = millis();
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case WALL_FRONT_LEFT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_LEFT;
        timerEvent = millis();
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case LINE_FRONT_RIGHT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_RIGHT;
        timerEvent = millis();
        if (lastEvent == WALL_FRONT_LEFT)
        {
          dir = CORNER_LEFT;
          cornerDir = BACKWARD;
          timerEvent = millis();
        }
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case WALL_FRONT_RIGHT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_RIGHT;
        timerEvent = millis();
        if (lastEvent == LINE_FRONT_LEFT)
        {
          dir = CORNER_RIGHT;
          cornerDir = BACKWARD;
          timerEvent = millis();
        }
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case WALL_SIDE_LEFT:
      {
        dir = FORWARD_RIGHT;
        timerEvent = millis();
        break;
      }

      case WALL_SIDE_RIGHT:
      {
        dir = FORWARD_LEFT;
        timerEvent = millis();
        break;
      }
      }
      break;
    }

    case FORWARD_LEFT:
    {
      if (millis() - timerLastEvent > LAST_EVENT_DELAY)
      {
        lastEvent = NONE;
        timerLastEvent = millis();
      }
      forward(FORWARD_LEFT_TRAVEL_SPEED);
      left();

      switch (event)
      {
      case LINE_FRONT_LEFT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_LEFT;
        timerEvent = millis();
        if (lastEvent == WALL_FRONT_RIGHT)
        {
          dir = CORNER_RIGHT;
          cornerDir = BACKWARD;
          timerEvent = millis();
        }
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case WALL_FRONT_LEFT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_LEFT;
        timerEvent = millis();
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case LINE_FRONT_RIGHT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_RIGHT;
        timerEvent = millis();
        if (lastEvent == WALL_FRONT_LEFT)
        {
          dir = CORNER_LEFT;
          cornerDir = BACKWARD;
          timerEvent = millis();
        }
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case WALL_FRONT_RIGHT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_RIGHT;
        timerEvent = millis();
        lastEvent = event;
        timerLastEvent = millis();
        break;
      }

      case WALL_SIDE_LEFT:
      {
        dir = FORWARD_RIGHT;
        timerEvent = millis();
        break;
      }

      case WALL_SIDE_RIGHT:
      {
        dir = FORWARD_LEFT;
        timerEvent = millis();
        break;
      }
      }
      break;
    }

    case FORWARD_RIGHT:
    {
      if (millis() - timerEvent > EVENT_DELAY)
      {
        straight();
        dir = FORWARD;
        timerEvent = millis();
        event = NONE;
        break;
      }
      right();

      switch (event)
      {
      case LINE_FRONT_LEFT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_LEFT;
        timerEvent = millis();
        break;
      }

      case LINE_FRONT_RIGHT:
      {
        backward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        dir = BACKWARD_RIGHT;
        timerEvent = millis();
        break;
      }
      }
      break;
    }

    case BACKWARD_LEFT:
    case BACKWARD_RIGHT:
    {
      if (millis() - timerEvent > EVENT_DELAY)
      {
        forward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        straight();
        dir = FORWARD;
        timerEvent = millis();
        event = NONE;
        break;
      }
      backward(BACKWARD_TRAVEL_SPEED);
      if (dir == BACKWARD_LEFT)
        left();
      else
        right();

      switch (event)
      {
      case LINE_REAR_LEFT:
      case LINE_REAR_RIGHT:
      {
        forward(BRAKE_SPEED); //BREAKING
        delay(BRAKE_DELAY);
        straight();
        dir = FORWARD;
        timerEvent = millis();
        break;
      }
      }
      break;
    }

    case CORNER_RIGHT:
    case CORNER_LEFT:
    {
      switch (cornerDir)
      {
      case FORWARD:
      {
        if (millis() - timerEvent < 100)
        forward(150);
      else
        forward(FORWARD_TRAVEL_SPEED);
 
        if (dir == CORNER_LEFT)
          right();
        else
          left();

        switch (event)
        {
        case LINE_FRONT_LEFT:
        {
          backward(BRAKE_SPEED); //BREAKING
          delay(BRAKE_DELAY);
          if (dir == CORNER_LEFT)
            dir = BACKWARD_LEFT; // go back to normal mode
          else
            cornerDir = BACKWARD;

          timerEvent = millis();
          break;
        }

        case WALL_FRONT_LEFT:
        case WALL_FRONT_RIGHT:
        {
          backward(BRAKE_SPEED); //BREAKING
          delay(BRAKE_DELAY);
          cornerDir = BACKWARD;
          timerEvent = millis();

          break;
        }

        case LINE_FRONT_RIGHT:
        {
          backward(BRAKE_SPEED); //BREAKING
          delay(BRAKE_DELAY);
          if (dir == CORNER_RIGHT)
            dir = BACKWARD_RIGHT; // go back to normal mode
          else
            cornerDir = BACKWARD;

          timerEvent = millis();
          break;
        }
        }
        break;
      }

      case BACKWARD:
      {
        if (millis() - timerEvent > EVENT_DELAY)
        {
          forward(BRAKE_SPEED); //BREAKING
          delay(BRAKE_DELAY);
          cornerDir = FORWARD;
          timerEvent = millis();
          event = NONE;
          break;
        }
        backward(BACKWARD_TRAVEL_SPEED);
        if (dir == CORNER_LEFT)
          left();
        else
          right();

        switch (event)
        {
        case LINE_REAR_LEFT:
        case LINE_REAR_RIGHT:
        {
          forward(BRAKE_SPEED); //BREAKING
          delay(BRAKE_DELAY);
          straight();
          cornerDir = FORWARD;
          timerEvent = millis();
          break;
        }
        }
        break;
      }

      default:
        dir = FORWARD;
        timerEvent = millis();
        break;
      }
      break;
    }
    }
  }
}
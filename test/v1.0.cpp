#include <Arduino.h>
#include <util/atomic.h>

#define ISR_STATUS_TRIGGERED 1
#define LASER_SENSOR_INTERRUPT_TYPE CHANGE
#define LASER_SENSOR_INTERRUPT_DEBOUNCE_TIME_MS 500

typedef struct TimerInfo_s {
    unsigned int TimerStarted: 1;
    unsigned int TimerCompleted: 1;
    unsigned long StartTime_ms;
    unsigned long EndTime_ms;
} TimerInfo_t;

volatile static TimerInfo_t TimerInfo_volatile = {};
static TimerInfo_t TimerInfo_temp = {};

int LaserSensorTriggerPin = 2;

void LaserSensor_ISR() {
    int LaserSensorValue = digitalRead(LaserSensorTriggerPin);
    unsigned long currentMillis = millis();

    if (TimerInfo_volatile.TimerStarted == 0 && LaserSensorValue == HIGH) {
        TimerInfo_volatile.TimerStarted = 1;
        TimerInfo_volatile.StartTime_ms = currentMillis;
    } else if (TimerInfo_volatile.TimerStarted == 1 && TimerInfo_volatile.TimerCompleted == 0 && LaserSensorValue == HIGH) {
        if ((currentMillis - TimerInfo_volatile.StartTime_ms) > LASER_SENSOR_INTERRUPT_DEBOUNCE_TIME_MS) {
            TimerInfo_volatile.TimerCompleted = 1;
            TimerInfo_volatile.EndTime_ms = currentMillis;
        }
    }
}

void setup() {
    pinMode(LaserSensorTriggerPin, INPUT);
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(LaserSensorTriggerPin), LaserSensor_ISR, LASER_SENSOR_INTERRUPT_TYPE);
}

void loop() {
    // Access the volatile struct atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TimerInfo_temp.TimerStarted = TimerInfo_volatile.TimerStarted;
        TimerInfo_temp.TimerCompleted = TimerInfo_volatile.TimerCompleted;
        TimerInfo_temp.StartTime_ms = TimerInfo_volatile.StartTime_ms;
        TimerInfo_temp.EndTime_ms = TimerInfo_volatile.EndTime_ms;
    }
    if (TimerInfo_temp.TimerStarted == 1 && TimerInfo_temp.TimerCompleted == 0) {
        Serial.print("Counting: ");
        Serial.println(((float)(millis() - TimerInfo_temp.StartTime_ms)) / 1000.0f, 3);
    } else if (TimerInfo_temp.TimerCompleted == 1) {
        Serial.print("End: ");
        Serial.println(((float)(TimerInfo_temp.EndTime_ms - TimerInfo_temp.StartTime_ms)) / 1000.0f, 3);

        // Reset timer for the next measurement
        /*ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            TimerInfo_volatile.TimerStarted = 0;
            TimerInfo_volatile.TimerCompleted = 0;
        }*/
    } else {
        Serial.println("Not started");
    }

    delay(100); // Adjust delay as necessary
}

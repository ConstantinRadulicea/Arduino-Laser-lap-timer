/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


#include <Arduino.h>
#include <util/atomic.h>

#define ISR_STATUS_TRIGGERED 1
#define LASER_SENSOR_INTERRUPT_TYPE CHANGE
#define LASER_SENSOR_INTERRUPT_DEBOUNCE_TIME_MS 500
#define TIMER_STATE_IDLE 0
#define TIMER_STATE_TIMING 1
#define TIMER_STATE_DONE 2

typedef struct TimerInfo_s {
    unsigned int State; // States: 0 - Idle, 1 - Timing, 2 - Done
    unsigned long StartTime_ms;
    unsigned long EndTime_ms;
} TimerInfo_t;

volatile static TimerInfo_t TimerInfo_volatile = {};
static TimerInfo_t TimerInfo_temp = {};

int LaserSensorTriggerPin = 2;

void LaserSensor_ISR() {
    int LaserSensorValue = digitalRead(LaserSensorTriggerPin);
    unsigned long currentMillis = millis();

    if (TimerInfo_volatile.State == TIMER_STATE_IDLE && LaserSensorValue == HIGH) {
        TimerInfo_volatile.State = TIMER_STATE_TIMING; // Transition to Timing state
        TimerInfo_volatile.StartTime_ms = currentMillis;
    }
    else if (TimerInfo_volatile.State == TIMER_STATE_TIMING && LaserSensorValue == HIGH) {
        if ((currentMillis - TimerInfo_volatile.StartTime_ms) > LASER_SENSOR_INTERRUPT_DEBOUNCE_TIME_MS) {
            TimerInfo_volatile.State = TIMER_STATE_DONE; // Transition to Done state
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
        TimerInfo_temp.State = TimerInfo_volatile.State;
        TimerInfo_temp.StartTime_ms = TimerInfo_volatile.StartTime_ms;
        TimerInfo_temp.EndTime_ms = TimerInfo_volatile.EndTime_ms;
    }

    switch (TimerInfo_temp.State) {
        case TIMER_STATE_IDLE:
            Serial.println("State: Idle");
            break;
        case TIMER_STATE_TIMING:
            Serial.print("State: Timing, Elapsed Time: ");
            Serial.println(((float)(millis() - TimerInfo_temp.StartTime_ms)) / 1000.0f, 3);
            break;
        case TIMER_STATE_DONE:
            Serial.print("State: Done, Duration: ");
            Serial.println(((float)(TimerInfo_temp.EndTime_ms - TimerInfo_temp.StartTime_ms)) / 1000.0f, 3);

            // Reset timer for the next measurement
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                TimerInfo_volatile.State = TIMER_STATE_IDLE; // Transition to Idle state
            }
            break;
        default:
            Serial.println("Unknown state");
            break;
    }

    delay(100); // Adjust delay as necessary
}

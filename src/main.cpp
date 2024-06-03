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
#include <LiquidCrystal_I2C.h>

#define LASER_SENSOR_INTERRUPT_TYPE CHANGE
#define RESET_BUTTON_INTERRUPT_TYPE CHANGE
#define LASER_SENSOR_INTERRUPT_DEBOUNCE_TIME_MS 1000
#define RESET_BUTTON_INTERRUPT_DEBOUNCE_TIME_MS 50
#define TIMER_STATE_IDLE 0
#define TIMER_STATE_TIMING 1
#define TIMER_STATE_DONE 2
#define LCD_MAX_FPS 10

#define LASER_SENSOR_TRIGGER_PIN 2
#define TIMER_RESET_BUTTON_PIN 3

typedef struct TimerInfo_s {
    uint8_t State; // States: 0 - Idle, 1 - Timing, 2 - Done
    unsigned long StartTime_ms;
    unsigned long EndTime_ms;
} TimerInfo_t;

volatile static TimerInfo_t TimerInfo_volatile = {};
static TimerInfo_t TimerInfo_temp = {};
LiquidCrystal_I2C lcd(0x3F, 16, 2); //0x27

void LaserSensor_ISR() {
    int LaserSensorValue = digitalRead(LASER_SENSOR_TRIGGER_PIN);
    unsigned long currentMillis = millis();

    static unsigned long lastInterruptTime = 0;

    if ((currentMillis - lastInterruptTime) > LASER_SENSOR_INTERRUPT_DEBOUNCE_TIME_MS) {
        lastInterruptTime = currentMillis;

        switch (TimerInfo_volatile.State) {
            case TIMER_STATE_IDLE:
                if (LaserSensorValue == HIGH) {
                    TimerInfo_volatile.State = TIMER_STATE_TIMING; // Transition to Timing state
                    TimerInfo_volatile.StartTime_ms = currentMillis;
                }
                break;
                
            case TIMER_STATE_TIMING:
                if (LaserSensorValue == HIGH) {
                    TimerInfo_volatile.State = TIMER_STATE_DONE; // Transition to Done state
                    TimerInfo_volatile.EndTime_ms = currentMillis;
                }
                break;
                
            case TIMER_STATE_DONE:
                // Do nothing, wait for reset
                break;
        }
    }
}

void ResetButton_ISR() {
    static unsigned long lastButtonPressTime = 0;
    unsigned long currentMillis = millis();

    // Debounce the reset button
    if ((currentMillis - lastButtonPressTime) > RESET_BUTTON_INTERRUPT_DEBOUNCE_TIME_MS) {
        lastButtonPressTime = currentMillis;
        
        // Reset timer if the button is pressed
        if (digitalRead(TIMER_RESET_BUTTON_PIN) == HIGH) {
            TimerInfo_volatile.State = TIMER_STATE_IDLE;
            TimerInfo_volatile.StartTime_ms = 0;
            TimerInfo_volatile.EndTime_ms = 0;
        }
    }
}

void setup() {
    pinMode(LASER_SENSOR_TRIGGER_PIN, INPUT);
    pinMode(TIMER_RESET_BUTTON_PIN, INPUT_PULLUP); // Internal pull-up resistor
    //Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(LASER_SENSOR_TRIGGER_PIN), LaserSensor_ISR, LASER_SENSOR_INTERRUPT_TYPE);
    attachInterrupt(digitalPinToInterrupt(TIMER_RESET_BUTTON_PIN), ResetButton_ISR, RESET_BUTTON_INTERRUPT_TYPE);
    lcd.init();  //display initialization
    lcd.backlight();  // activate the backlight
}


float SecondsPassed = 0.0f;

void loop() {
    // Access the volatile struct atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TimerInfo_temp.State = TimerInfo_volatile.State;
        TimerInfo_temp.StartTime_ms = TimerInfo_volatile.StartTime_ms;
        TimerInfo_temp.EndTime_ms = TimerInfo_volatile.EndTime_ms;
    }
    lcd.clear();
    switch (TimerInfo_temp.State) {
        case TIMER_STATE_IDLE:
            //Serial.println("State: Idle");
            lcd.setCursor(0, 0);
            lcd.print("Idle");
            break;
        case TIMER_STATE_TIMING:
            SecondsPassed = ((float)(millis() - TimerInfo_temp.StartTime_ms)) / 1000.0f;
            //Serial.print("State: Timing, Elapsed Time: ");
            //Serial.println(SecondsPassed, 3);

            lcd.setCursor(0, 0);
            lcd.print("Timing [s]");
            lcd.setCursor(0, 1);
            lcd.print(SecondsPassed, 3);
            break;
        case TIMER_STATE_DONE:
            SecondsPassed = ((float)(TimerInfo_temp.EndTime_ms - TimerInfo_temp.StartTime_ms)) / 1000.0f;
            //Serial.print("State: Done, Duration: ");
            //Serial.println(SecondsPassed, 3);
            lcd.setCursor(0, 0);
            lcd.print("Done [s]");
            lcd.setCursor(0, 1);
            lcd.print(SecondsPassed, 3);
            break;
        default:
            //Serial.println("Unknown state");
            lcd.setCursor(0, 0);
            lcd.print("Unknown state");
            break;
    }

    delay((unsigned long)(1000.0f / (float)LCD_MAX_FPS)); // Adjust delay as necessary
}

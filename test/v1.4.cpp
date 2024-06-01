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
int TimerResetButtonPin = 3;

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

void ResetButton_ISR() {
    // Reset timer if the button is pressed
    if (digitalRead(TimerResetButtonPin) == HIGH) {
        TimerInfo_volatile.State = TIMER_STATE_IDLE;
        TimerInfo_volatile.StartTime_ms = 0;
        TimerInfo_volatile.EndTime_ms = 0;
    }
}

void setup() {
    pinMode(LaserSensorTriggerPin, INPUT);
    pinMode(TimerResetButtonPin, INPUT_PULLUP); // Internal pull-up resistor
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(LaserSensorTriggerPin), LaserSensor_ISR, LASER_SENSOR_INTERRUPT_TYPE);
    attachInterrupt(digitalPinToInterrupt(TimerResetButtonPin), ResetButton_ISR, RISING);
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
            break;
        default:
            Serial.println("Unknown state");
            break;
    }

    delay(100); // Adjust delay as necessary
}

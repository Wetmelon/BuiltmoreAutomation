#include "Builtmore_Defines.h"
#include <DAC_MCP49xx.h>
#include <AccelStepper.h>

typedef enum {
    BA_STATE_IDLE = 0,
    BA_STATE_ERROR,
    BA_STATE_INIT,
    BA_STATE_WAITING,
} ba_state_t;

AccelStepper stepper(1, STEP1_PIN, DIR1_PIN);
DAC_MCP49xx myDac(DAC_MCP49xx::MCP4921, CS_PIN);
ba_state_t eState;

uint32_t timer = 0;

void setup(){
    
    eState = BA_STATE_INIT;

    // Prep uC Pins
    pinMode(EN1_PIN, OUTPUT);
    pinMode(STEP1_PIN, OUTPUT);
    pinMode(DIR1_PIN, OUTPUT);
    pinMode(EN2_PIN, OUTPUT);
    pinMode(STEP2_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);

    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);
    pinMode(ENC_Z_PIN, INPUT);

    pinMode(CS_PIN, OUTPUT);

    pinMode(LIMIT1_PIN, INPUT_PULLUP);
    pinMode(LIMIT2_PIN, INPUT_PULLUP);
    pinMode(LIMIT3_PIN, INPUT_PULLUP);
    pinMode(LIMIT4_PIN, INPUT_PULLUP);

    digitalWriteFast(EN1_PIN, HIGH);
    digitalWriteFast(STEP1_PIN, LOW);
    digitalWriteFast(DIR1_PIN, LOW);
    digitalWriteFast(EN2_PIN, HIGH);
    digitalWriteFast(STEP2_PIN, LOW);
    digitalWriteFast(DIR2_PIN, LOW);

    digitalWriteFast(CS_PIN, HIGH);

    Serial.begin(250000);
    delay(1000);
    Serial.println("Serial Starting.");

    analogReadResolution(12);
    analogReadAveraging(4);

    myDac.output(0);
    
    stepper.setMaxSpeed(2000);
    stepper.setSpeed(500);
}

void loop(){

    positionControl();
    //speedControl();

}

void testDAC(){
    static uint16_t dacVal = 0;
    myDac.output(dacVal++);
    Serial.print("DacVal: ");
    Serial.print(dacVal);
    Serial.print("\tAnalog: ");
    Serial.println(analogRead(A4));
    if (dacVal >= 4095)
        dacVal = 0;
}

void speedControl(){
    stepper.runSpeed();
}

void positionControl(){
    if (stepper.distanceToGo() == 0)
    {
        // Random change to speed, position and acceleration
        // Make sure we dont get 0 speed or accelerations
        Serial.print("Timer: ");
        Serial.println(millis() - timer);
        delay(1000);
        int move = rand() % 200;
        int speed = rand() % 200;
        int accel = rand() % 200;

        move = 20000;
        speed = 1000;
        accel = 500;

        stepper.move(move);
        stepper.setMaxSpeed(speed);
        stepper.setAcceleration(accel);

        Serial.print("Move: ");
        Serial.print(stepper.targetPosition());
        Serial.print("\tDiff: ");
        Serial.print(stepper.distanceToGo());
        Serial.print("\tSpeed: ");
        Serial.print(stepper.maxSpeed());
        Serial.print("\tAccel: ");
        Serial.println(accel);
        timer = millis();
    }
    
    stepper.run();
    //delayMicroseconds(1);
}
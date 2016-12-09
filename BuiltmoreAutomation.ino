
#include "Builtmore_Defines.h"
#include <MultiStepper.h>
#include <AccelStepper.h>
#include <DAC_MCP49xx.h>

typedef enum {
    BA_STATE_IDLE = 0,
    BA_STATE_ERROR,
    BA_STATE_INIT,
    BA_STATE_SWITCH1,
    BA_STATE_SWITCH2,
    BA_STATE_STAMPER_ONLY,
    BA_STATE_BOTH,
    BA_STATE_REVERSE,
} ba_state_t;

AccelStepper stamperStepper(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper seederStepper(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);

DAC_MCP49xx myDac(DAC_MCP49xx::MCP4921, CS_PIN);
ba_state_t state = BA_STATE_INIT;

uint32_t seederTimer = 0;
volatile uint32_t speedTimer = 0;
volatile float traySpeed = 0;

uint32_t move = 0;
uint32_t accel = 1;

void setup(){
    state = BA_STATE_INIT;

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

    attachInterrupt(digitalPinToInterrupt(14), speedSwitchOneHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(15), speedSwitchTwoHandler, FALLING);

    Serial.begin(250000);
    delay(1000);
    Serial.println("Serial Starting.");

    analogReadResolution(12);
    analogReadAveraging(4);

    stamperStepper.setSpeed(0);
    seederStepper.setSpeed(0);

    stamperStepper.setCurrentPosition(0);
    seederStepper.setCurrentPosition(0);

    myDac.output(DESIRED_SPEED);

    state = BA_STATE_IDLE;
}

void loop(){

    switch (state){
    case BA_STATE_STAMPER_ONLY:
        if (millis() - seederTimer > SEEDER_DELAY){
            seederStepper.setSpeed(traySpeed / DISC_RADIUS_M * 100 / (PI) *  MICROSTEPPING * PULLEY_RATIO);
            state = BA_STATE_BOTH;
        }
            stamperStepper.run();
        break;

    case BA_STATE_BOTH:
        if (stamperStepper.distanceToGo() == 0){
            stamperStepper.moveTo(0);
            state = BA_STATE_REVERSE;
        }
        else {
            seederStepper.runSpeed();
            stamperStepper.run();
        }
        break;

    case BA_STATE_REVERSE:
        if (stamperStepper.distanceToGo() == 0){
            seederStepper.setSpeed(0);
            state = BA_STATE_IDLE;
        }
        else {
            seederStepper.runSpeed();
            stamperStepper.run();
        }
        break;

    case BA_STATE_INIT:     setup(); break;
    case BA_STATE_SWITCH2:  startRunning(); break;
    case BA_STATE_SWITCH1:
    case BA_STATE_IDLE:
    case BA_STATE_ERROR:
    default: break;
    }
}

void startRunning(){
    if (state == BA_STATE_SWITCH2){
#ifdef DEBUG
        Serial.print("Timer: ");
        Serial.println(millis() - timer);
#endif

        delay(STAMPER_DELAY);

        stamperStepper.setMaxSpeed(traySpeed / DISC_RADIUS_M * 100 / (PI)*  MICROSTEPPING * PULLEY_RATIO);
        stamperStepper.setAcceleration(STEPPER_ACCEL * MICROSTEPPING * PULLEY_RATIO);
        stamperStepper.move(FORWARD_STEPS * MICROSTEPPING * PULLEY_RATIO);

        state = BA_STATE_STAMPER_ONLY;

#ifdef DEBUG
        Serial.print("Move: ");
        Serial.print(stamperStepper.targetPosition());
        Serial.print("\tDiff: ");
        Serial.print(stamperStepper.distanceToGo());
        Serial.print("\tSpeed: ");
        Serial.print(stamperStepper.maxSpeed());
        Serial.print("\tAccel: ");
        Serial.println(STEPPER_ACCEL);

        timer = millis();
#endif
    }
}

void speedSwitchOneHandler(){
    if (state == BA_STATE_IDLE){
        speedTimer = millis();
        state = BA_STATE_SWITCH1;
    }
}

void speedSwitchTwoHandler(){
    if (state == BA_STATE_SWITCH1){
        traySpeed = SWITCH_DISTANCE_M / (millis() - speedTimer);
        state = BA_STATE_SWITCH2;
    }
}
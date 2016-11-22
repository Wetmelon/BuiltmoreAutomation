#include "Builtmore_Defines.h"
#include <DAC_MCP49xx.h>
#include <AccelStepper.h>

DAC_MCP49xx myDac(DAC_MCP49xx::MCP4921, CS_PIN);

void setup(){
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

    myDac.output(0);
}

void loop(){
    delay(100);
}
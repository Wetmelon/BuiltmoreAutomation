#include <MultiStepper.h>
#include <AccelStepper.h>
#include "pins_RAMPS.h"
#include "macros.h"

AccelStepper gantryStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);      // Gantry
AccelStepper stampStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);      // Stamper
AccelStepper seedStepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);      // Seeder

void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);
    pinMode(Z_ENABLE_PIN, OUTPUT);

    digitalWrite(X_ENABLE_PIN, LOW);
    digitalWrite(Y_ENABLE_PIN, LOW);
    digitalWrite(Z_ENABLE_PIN, LOW);

    gantryStepper.setPinsInverted(false, false, true);
    stampStepper.setPinsInverted(false, false, true);
    seedStepper.setPinsInverted(false, false, true);

    gantryStepper.setMaxSpeed(12000);
    gantryStepper.setAcceleration(7500);
    stampStepper.setMaxSpeed(100);
    stampStepper.setAcceleration(20);
    seedStepper.setMaxSpeed(100);
    seedStepper.setAcceleration(20);
}

void loop()
{
    if (gantryStepper.distanceToGo() == 0)
    {
        // Random change to position
        delay(1000);
        gantryStepper.moveTo((long)rand() % 2000*16);
    }
    gantryStepper.run();
}

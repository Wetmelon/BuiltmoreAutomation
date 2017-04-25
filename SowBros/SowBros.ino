#include <MultiStepper.h>
#include <AccelStepper.h>
#include "pins_RAMPS.h"
#include "macros.h"

#define MICROSTEPPING       (16L)
#define X_STEPS_PER_REV     (200L*MICROSTEPPING)
#define Z_STEPS_PER_REV     (200L*MICROSTEPPING)
#define E1_STEPS_PER_REV    (200L*MICROSTEPPING)

#define X_MM_PER_REV        (60L)    // GT2 belt, 30 tooth pulley
#define Z_MM_PER_REV        (60L)    // INCORRECT!  Check leadscrew pitch!
#define E1_MM_PER_REV       (60L)

#define X_MM_TO_STEPS(x)    ((x)/(X_MM_PER_REV*X_STEPS_PER_REV))
#define Z_MM_TO_STEPS(x)    ((x)/(Z_MM_PER_REV*Z_STEPS_PER_REV))
#define E1_MM_TO_STEPS(x)   ((x)/(E1_MM_PER_REV*E1_STEPS_PER_REV))

#define X_HOME_SPEED        (50)    // in mm/s
#define Z_HOME_SPEED        (50)

#define FIRST_TRAY_POSITION     (200)   // in mm
#define SECOND_TRAY_POSITION    (1000)
#define THIRD_TRAY_POSITION     (1800)

#define RELATIVE_ROW_DIST   (53)    // in mm
#define STAMP_DIST          (50)
#define SEED_DIST           (50)

AccelStepper gantryStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);    // Gantry
AccelStepper stampStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);     // Stamper
AccelStepper seedStepper(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN);    // Seeder

enum state {
    INIT,
    HOME,
    RUNNING,
    FINAL,
    PAUSE
};

enum moveState {
    NONE,
    GANTRY,
    STAMP_SEED,
    STAMPER,
    SEEDER
};

enum state myState = INIT;
enum state lastState = INIT;

void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Z_ENABLE_PIN, OUTPUT);
    pinMode(E1_ENABLE_PIN, OUTPUT);

    digitalWrite(X_ENABLE_PIN, HIGH);    // Disable all three stepper drivers
    digitalWrite(Z_ENABLE_PIN, HIGH);
    digitalWrite(E1_ENABLE_PIN, HIGH);

    gantryStepper.setMaxSpeed(X_MM_TO_STEPS(225));          // 225 mm/s max speed
    gantryStepper.setAcceleration(X_MM_TO_STEPS(140));     // 140 mm/s^2 max acceleration

    stampStepper.setMaxSpeed(100);
    stampStepper.setAcceleration(20);

    seedStepper.setMaxSpeed(100);
    seedStepper.setAcceleration(20);
}

// Main loop.  No functions block, so this runs very often
void loop()
{
    static bool lastButtonState = INIT;
    static bool buttonState = INIT;

    // Multi-purpose button starts the device and pauses it.
    // Pressing the button again resumes motion.
    buttonState = !digitalRead(Y_MIN_PIN);
    if (buttonState && !lastButtonState)
    {
        lastButtonState = buttonState;

        lastState = myState;
        switch (myState){
        case INIT: myState = HOME; break;
        case HOME: myState = PAUSE; break;
        case RUNNING: myState = PAUSE; break;
        case FINAL: myState = PAUSE; break;
        case PAUSE: myState = lastState; break;
        }
    }
    else if (!buttonState && lastButtonState)
        lastButtonState = buttonState;

    // Called every loop
    switch (myState){
    case INIT: initState(); break;
    case HOME: homeState(); break;
    case RUNNING: runState(); break;
    case PAUSE: pauseState(); break;
    case FINAL: finalState(); break;
    default: initState();
    }
}

void initState(){
    // In INIT, nothing should be moving.
    // Disable all steppers and stop them.
    digitalWrite(X_ENABLE_PIN, HIGH);
    digitalWrite(Z_ENABLE_PIN, HIGH);
    digitalWrite(E1_ENABLE_PIN, HIGH);

    gantryStepper.stop();
    stampStepper.stop();
    seedStepper.stop();
}

// Home the axes!
void homeState(){
    static bool gantryHomeFlag = !digitalRead(X_MIN_PIN);
    static bool stamperHomeFlag = !digitalRead(Z_MIN_PIN);

    gantryHomeFlag = !digitalRead(X_MIN_PIN);
    stamperHomeFlag = !digitalRead(Z_MIN_PIN);

    if (gantryHomeFlag){
        gantryStepper.stop();
        gantryStepper.setSpeed(0);
    }

    if (stamperHomeFlag){
        stampStepper.stop();
        stampStepper.setSpeed(0);
    }

    if (gantryHomeFlag && stamperHomeFlag)
    {
        myState = RUNNING;
        gantryStepper.setCurrentPosition(0);    // We're home.  Reset step counts.
        stampStepper.setCurrentPosition(0);
    }
    else
    {
        gantryStepper.runSpeed();
        stampStepper.runSpeed();
    }
}

// Called after the machine has homed
void runState(){
    static int rowCount = 0;
    static int trayCount = 0;

    static enum moveState nextMove = GANTRY;

    if (gantryStepper.distanceToGo() == 0 && stampStepper.distanceToGo() == 0 && seedStepper.distanceToGo() == 0){    // Nothing Moving
        if (nextMove == GANTRY){
            if (rowCount++ == 0) {
                if (trayCount++ == 0)
                    gantryStepper.moveTo(X_MM_TO_STEPS(FIRST_TRAY_POSITION));
                else if (trayCount++ == 1)
                    gantryStepper.moveTo(X_MM_TO_STEPS(SECOND_TRAY_POSITION));
                else if (trayCount++ == 2)
                    gantryStepper.moveTo(X_MM_TO_STEPS(THIRD_TRAY_POSITION));
                else {
                    myState = FINAL;
                    return;
                }
                nextMove = STAMPER;
            }
            else if (rowCount++ < 2){
                gantryStepper.move(X_MM_TO_STEPS(RELATIVE_ROW_DIST));
                nextMove = STAMPER;
            }
            else if (rowCount++ < 5){
                gantryStepper.move(X_MM_TO_STEPS(RELATIVE_ROW_DIST));
                nextMove = STAMP_SEED;
            }
            else if (rowCount++ < 7){
                gantryStepper.move(X_MM_TO_STEPS(RELATIVE_ROW_DIST));
                nextMove = SEEDER;
            }
            else
                rowCount = 0;
        }

        else if (nextMove == STAMPER){
            if (stampStepper.currentPosition() != 0) {
                stampStepper.moveTo(0);
                nextMove = GANTRY;
            }
            else {
                stampStepper.moveTo(Z_MM_TO_STEPS(STAMP_DIST));
                nextMove = STAMPER; // Need to raise the stamper
            }
        }

        else if (nextMove == STAMP_SEED){
            seedStepper.moveTo(E1_MM_TO_STEPS(SEED_DIST));
            stampStepper.moveTo(Z_MM_TO_STEPS(STAMP_DIST));
            nextMove = STAMPER; // Need to raise the stamper
        }

        else if (nextMove == SEEDER){
            seedStepper.moveTo(E1_MM_TO_STEPS(SEED_DIST));
            nextMove = GANTRY;
        }
    }

    gantryStepper.run();
    stampStepper.run();
    seedStepper.run();
}

void finalState(){
    static bool gantryHomeFlag = !digitalRead(X_MIN_PIN);
    static bool stamperHomeFlag = !digitalRead(Z_MIN_PIN);

    gantryHomeFlag = !digitalRead(X_MIN_PIN);
    stamperHomeFlag = !digitalRead(Z_MIN_PIN);

    if (gantryHomeFlag){
        gantryStepper.stop();
        gantryStepper.setSpeed(0);
    }

    if (stamperHomeFlag){
        stampStepper.stop();
        stampStepper.setSpeed(0);
    }

    if (gantryHomeFlag && stamperHomeFlag)
    {
        myState = INIT;
        gantryStepper.setCurrentPosition(0);    // We're home.  Reset step counts.
        stampStepper.setCurrentPosition(0);
    }
    else
    {
        gantryStepper.runSpeed();
        stampStepper.runSpeed();
    }
}

void pauseState(){
    // Do nothing.  Keep everything enabled, but don't call run() for the steppers.
}
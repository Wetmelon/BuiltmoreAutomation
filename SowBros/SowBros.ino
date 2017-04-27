#include <MultiStepper.h>
#include <AccelStepper.h>
#include "pins_RAMPS.h"
#include "macros.h"

#define MICROSTEPPING       (16L)
#define X_STEPS_PER_REV     (200L*MICROSTEPPING)
#define Z_STEPS_PER_REV     (200L*MICROSTEPPING)
#define E1_STEPS_PER_REV    (200L*MICROSTEPPING)

#define X_MAX_SPEED         (500)
#define Z_MAX_SPEED         (500)
#define E1_MAX_SPEED        (200)

#define X_MM_PER_REV        (60L)    // GT2 belt, 30 tooth pulley
#define Z_MM_PER_REV        (60L)    // INCORRECT!  Check leadscrew pitch!
#define E1_MM_PER_REV       (60L)

#define X_MM_TO_STEPS(x)    ((x)/60.0*3200.0)
#define Z_MM_TO_STEPS(x)    ((x)/8.0*3200.0)
#define E1_MM_TO_STEPS(x)   ((x)*3200.0)

#define X_HOME_SPEED        (50)    // in mm/s
#define Z_HOME_SPEED        (50)

#define FIRST_TRAY_POSITION     (132)   // in mm
#define SECOND_TRAY_POSITION    (448)
#define THIRD_TRAY_POSITION     (762)

#define RELATIVE_ROW_DIST   (52)    // in mm
#define STAMP_DIST          (30)
#define SEED_DIST           (.125)

AccelStepper gantryStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);    // Gantry
AccelStepper stampStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);     // Stamper
AccelStepper seedStepper(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN);    // Seeder

enum state {
    INIT,
    HOME,
    RUNNING,
    FINAL,
    WAITING,
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
    Serial.begin(115200);
    delay(100);
    Serial.println("Starting!");

    pinMode(13, OUTPUT);
    pinMode(X_MAX_PIN, INPUT_PULLUP);
    pinMode(X_MIN_PIN, INPUT_PULLUP);
    pinMode(Y_MAX_PIN, INPUT_PULLUP);
    pinMode(Y_MIN_PIN, INPUT_PULLUP);
    pinMode(Z_MAX_PIN, INPUT_PULLUP);
    pinMode(Z_MIN_PIN, INPUT_PULLUP);
    digitalWrite(13, HIGH);

    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Z_ENABLE_PIN, OUTPUT);
    pinMode(E1_ENABLE_PIN, OUTPUT);

    digitalWrite(X_ENABLE_PIN, HIGH);    // Enable all three stepper drivers
    digitalWrite(Z_ENABLE_PIN, HIGH);
    digitalWrite(E1_ENABLE_PIN, HIGH);

    gantryStepper.setMaxSpeed(X_MM_TO_STEPS(X_MAX_SPEED));          // 225 mm/s max speed
    gantryStepper.setAcceleration(3000);     // 140 mm/s^2 max acceleration

    stampStepper.setMaxSpeed(Z_MAX_SPEED);
    stampStepper.setAcceleration(3000);

    seedStepper.setMaxSpeed(E1_MAX_SPEED);
    seedStepper.setAcceleration(1000);
}

// Main loop.  No functions block, so this runs very often
void loop()
{
    static bool lastButtonState = LOW;
    static bool buttonState = HIGH;

    // Multi-purpose button starts the device and pauses it.
    // Pressing the button again resumes motion.
    buttonState = !digitalRead(Y_MIN_PIN);
    
   /* Serial.print("X_Max:\t");
    Serial.println(digitalRead(X_MAX_PIN));
    Serial.println(digitalRead(X_MIN_PIN));
    Serial.println(digitalRead(Y_MAX_PIN));
    Serial.println(digitalRead(Y_MIN_PIN));
    Serial.println(digitalRead(Z_MAX_PIN));
    Serial.println(digitalRead(Z_MIN_PIN));*/
    //Serial.println(buttonState);
    if (buttonState && !lastButtonState)
    {
        Serial.println("Button pressed!");
        lastButtonState = buttonState;
        
        switch (myState){
        case INIT: 
            myState = HOME;
            digitalWrite(X_ENABLE_PIN, LOW);
            digitalWrite(Z_ENABLE_PIN, LOW);
            digitalWrite(E1_ENABLE_PIN, LOW);
            Serial.println("Entering HOME State");
            lastState = INIT;
            break;
        case HOME: 
            myState = HOME; 
            Serial.println("Entering HOME state.");  
            lastState = HOME;
            break;
        case RUNNING: 
            myState = RUNNING;
            Serial.println("Entering RUNNING state.");
            lastState = RUNNING;
            break;
        case FINAL:
            myState = FINAL; 
            Serial.println("Entering FINAL state"); 
            lastState = FINAL;
            break;
        case WAITING:
            myState = RUNNING;
            Serial.println("Entering RUNNING state");
            break;
        case PAUSE: 
            myState = lastState;
            Serial.println("Returning to last state.");
            break;
        }
    }
    else if (!buttonState && lastButtonState)
        lastButtonState = buttonState;

    // Called every loop
    //Serial.println("Checking State.");
    switch (myState){
    case INIT: initState(); break;
    case HOME: homeState(); break;
    case RUNNING: runState(); break;
    case PAUSE: pauseState(); break;
    case FINAL: finalState(); break;
    case WAITING: break;
    default: initState();
    }
    //delay(100);
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
    //Serial.println("Homing.");
    static bool gantryHomeFlag = !digitalRead(X_MIN_PIN);
    static bool stamperHomeFlag = !digitalRead(Z_MIN_PIN);

    if (!gantryHomeFlag)
        gantryHomeFlag = !digitalRead(X_MIN_PIN);
    if (!stamperHomeFlag)
        stamperHomeFlag = !digitalRead(Z_MAX_PIN);
    /*Serial.print("Gantry: ");
    Serial.println(gantryHomeFlag);
    Serial.print("Stamper: ");
    Serial.println(stamperHomeFlag);*/

    if (gantryHomeFlag){
        Serial.print("Gantry low");
        gantryStepper.stop();
        gantryStepper.setCurrentPosition(0);
    }
    else if (gantryStepper.distanceToGo() == 0 && stamperHomeFlag){
        Serial.print("Moving gantry at: ");
        Serial.println(X_MM_TO_STEPS(X_HOME_SPEED));
        gantryStepper.setMaxSpeed(X_MM_TO_STEPS(X_HOME_SPEED));
        gantryStepper.move(X_MM_TO_STEPS(-1500));
    }

    if (stamperHomeFlag){
        //Serial.println("Stamper low");
        stampStepper.stop();
        stampStepper.setCurrentPosition(0);
    }
    else if (stampStepper.distanceToGo() == 0){
        Serial.println("Moving stamper at: ");
        Serial.println(Z_MM_TO_STEPS(Z_HOME_SPEED));
        stampStepper.setMaxSpeed(Z_MM_TO_STEPS(Z_HOME_SPEED));
        stampStepper.move(Z_MM_TO_STEPS(-1500));
    }

    if (gantryHomeFlag && stamperHomeFlag)
    {
        myState = WAITING;
        gantryStepper.setCurrentPosition(0);    // We're home.  Reset step counts.
        stampStepper.setCurrentPosition(0);
        gantryHomeFlag = false;
        stamperHomeFlag = false;
        gantryStepper.setMaxSpeed(X_MM_TO_STEPS(X_MAX_SPEED));
        stampStepper.setMaxSpeed(Z_MM_TO_STEPS(Z_MAX_SPEED));
    }
    else
    {
        //Serial.println(gantryStepper.speed());
        gantryStepper.run();
        stampStepper.run();
    }
}

// Called after the machine has homed
void runState(){
    //Serial.println("Running.");
    static int rowCount = 0;
    static int trayCount = 0;

    static enum moveState nextMove = GANTRY;

    if (gantryStepper.distanceToGo() == 0 && stampStepper.distanceToGo() == 0 && seedStepper.distanceToGo() == 0){    // Nothing Moving
        if (nextMove == GANTRY){
            Serial.println(rowCount);
                Serial.println(trayCount);
            if (rowCount == 0) {
                if (trayCount == 0)
                    gantryStepper.moveTo(X_MM_TO_STEPS(FIRST_TRAY_POSITION));
                else if (trayCount == 1)
                    gantryStepper.moveTo(X_MM_TO_STEPS(SECOND_TRAY_POSITION));
                else if (trayCount == 2)
                    gantryStepper.moveTo(X_MM_TO_STEPS(THIRD_TRAY_POSITION));
                else {
                    rowCount = 0;
                    trayCount = 0;
                    myState = FINAL;
                    return;
                }
                rowCount++;
                trayCount++;
                nextMove = STAMPER;
            }
            else if (rowCount < 2){
                Serial.print("Moving to row: ");
                Serial.println(rowCount);
                gantryStepper.move(X_MM_TO_STEPS(RELATIVE_ROW_DIST));
                rowCount++;
                nextMove = STAMPER;
            }
            else if (rowCount < 5){
                Serial.print("Moving to row: ");
                gantryStepper.move(X_MM_TO_STEPS(RELATIVE_ROW_DIST));
                rowCount++;
                nextMove = STAMP_SEED;
            }
            else if (rowCount < 7){
                gantryStepper.move(X_MM_TO_STEPS(RELATIVE_ROW_DIST));
                rowCount++;
                nextMove = SEEDER;
            }
            else
                rowCount = 0;
        }

        else if (nextMove == STAMPER){
            if (stampStepper.currentPosition() != 0) {
                Serial.println("Retracting");
                stampStepper.moveTo(0);
                nextMove = GANTRY;
            }
            else {
                Serial.println("Stamping.");
                stampStepper.moveTo(Z_MM_TO_STEPS(STAMP_DIST));
                nextMove = STAMPER; // Need to raise the stamper
            }
        }

        else if (nextMove == STAMP_SEED){
            Serial.println("Seed/stamp.");
            seedStepper.move(E1_MM_TO_STEPS(SEED_DIST));
            stampStepper.moveTo(Z_MM_TO_STEPS(STAMP_DIST));
            nextMove = STAMPER; // Need to raise the stamper
        }

        else if (nextMove == SEEDER){
            Serial.println("Seeding");
            seedStepper.move(E1_MM_TO_STEPS(SEED_DIST));
            nextMove = GANTRY;
        }
    }

    gantryStepper.run();
    stampStepper.run();
    seedStepper.run();
}

void finalState(){
    //Serial.println("Homing.");
    static bool gantryHomeFlag = !digitalRead(X_MIN_PIN);
    static bool stamperHomeFlag = !digitalRead(Z_MIN_PIN);

    if (!gantryHomeFlag)
        gantryHomeFlag = !digitalRead(X_MIN_PIN);
    if (!stamperHomeFlag)
        stamperHomeFlag = !digitalRead(Z_MAX_PIN);
    /*Serial.print("Gantry: ");
    Serial.println(gantryHomeFlag);
    Serial.print("Stamper: ");
    Serial.println(stamperHomeFlag);*/

    if (gantryHomeFlag){
        Serial.print("Gantry low");
        gantryStepper.stop();
        gantryStepper.setCurrentPosition(0);
    }
    else if (gantryStepper.distanceToGo() == 0 && stamperHomeFlag){
        Serial.print("Moving gantry at: ");
        Serial.println(X_MM_TO_STEPS(X_HOME_SPEED));
        gantryStepper.setMaxSpeed(X_MM_TO_STEPS(X_HOME_SPEED));
        gantryStepper.move(X_MM_TO_STEPS(-1500));
    }

    if (stamperHomeFlag){
        //Serial.println("Stamper low");
        stampStepper.stop();
        stampStepper.setCurrentPosition(0);
    }
    else if (stampStepper.distanceToGo() == 0){
        Serial.println("Moving stamper at: ");
        Serial.println(Z_MM_TO_STEPS(Z_HOME_SPEED));
        stampStepper.setMaxSpeed(Z_MM_TO_STEPS(Z_HOME_SPEED));
        stampStepper.move(Z_MM_TO_STEPS(-1500));
    }

    if (gantryHomeFlag && stamperHomeFlag)
    {
        myState = INIT;
        gantryStepper.setCurrentPosition(0);    // We're home.  Reset step counts.
        stampStepper.setCurrentPosition(0);
        gantryHomeFlag = false;
        stamperHomeFlag = false;
        gantryStepper.setMaxSpeed(X_MM_TO_STEPS(X_MAX_SPEED));
        stampStepper.setMaxSpeed(Z_MM_TO_STEPS(Z_MAX_SPEED));
    }
    else
    {
        //Serial.println(gantryStepper.speed());
        gantryStepper.run();
        stampStepper.run();
    }
}

void pauseState(){
    // Do nothing.  Keep everything enabled, but don't call run() for the steppers.
}
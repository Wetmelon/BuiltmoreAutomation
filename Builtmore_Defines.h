// Builtmore_Defines.h

#ifndef _BUILTMORE_DEFINES_h
#define _BUILTMORE_DEFINES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#define EN1_PIN     (0)
#define STEP1_PIN   (1)
#define DIR1_PIN    (2)
#define ENC_A_PIN   (3)
#define ENC_B_PIN   (4)
#define ENC_Z_PIN   (5)
#define EN2_PIN     (6)
#define STEP2_PIN   (7)
#define DIR2_PIN    (8)
#define LIMIT1_PIN  (14)
#define LIMIT2_PIN  (15)
#define LIMIT3_PIN  (16)
#define LIMIT4_PIN  (17)
#define CS_PIN      (10)


#endif


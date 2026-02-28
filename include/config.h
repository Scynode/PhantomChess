#ifndef CONFIGURE_H
#define CONFIGURE_H
//------------------------------------------------------------------------------


//---------------------------------------------------------------
#define myDebug

//#define _realizaJugada            //Enables comments in the realizaJugada function
//#define activateSensors             //Enables the sensors
//#define debugSensors              //Defines whether sensor output is constantly displayed

//#define generalDebug

//#define testWorkSpace               //Enables an initial traverse around the work area
//#define testPiecesAndCore           //Enables the traverse of the electromagnet turned off beneath the pieces in their initial positions

#define stateMachineActivate        //The program automatically enters the state machine

//#define modoDemostracion
//#define testSensoresSoloCambios    //This test only prints to serial each time something changes on the board; the debugSensors define must be commented out

//#define automaticInitSculpture      //The program starts playback automatically (Sculpture mode)
#define SelectModeBluetooth
//#define automaticInitPlayMode

//===================================================================================
//To enter play mode, comment out the previous automaticInitSculpture definition and uncomment
//the following lines to automatically skip some of the verifications
//---------------------------------------------------------------------------------- 
#define passReceiveInitialFen
#define passCheckMovement
#define passCompareEngineMechanism
#define passCheckMate


//#define CambiosPCBLalo
#define CambiosPCBTavo

//====================================================================================
//#define normalElectro                         
#define invertElectro                           //For Lalo it is invert Electro

#define pwmMagnet 0                             //PWM level, 0 is the maximum duty cycle

//#define activarTestSensors   //for sensor testing via Bluetooth React Native app
#define completeTest         //To run the complete test, electromagnets, motors, buzzer
//#define testTemp

#define version4Electro
//#define version1Electro

//#define curvasV1
#define curvasV2


//#define pinoutv1                //Configuration of the pinout for the first PCB
#define pinoutv2                //Configuration of the pinout for my PCB
//#define pinoutv3                    //Configuration of the pinout for Lalo's PCB

#ifdef pinoutv1

#define magnet1 13
#define magnet2 12
#define magnet3 22
#define magnet4 23

#endif

#ifdef pinoutv2

#define magnet1 13      //13
#define magnet2 12      //12
#define magnet3 22      //22
#define magnet4 23      //23

#endif

#ifdef pinoutv3

#define magnet1 19     //19 
#define magnet2 22     //23
#define magnet3 21     //21
#define magnet4 23     //22

#endif


//#define electromagnetConf1    //Electromagnet configuration for previous boards
#define electromagnetConf2    //Electromagnet configuration for new boards 


//#define checkArea                 //Enables square movement over the squares

#define centrarPiezas               //Enables the function to center pieces in their initial positions



//===================================================
//COMMUNICATION CONFIGURATION FOR PLAY MODE
//---------------------------------------------------
//#define receiveSerialMessages
#define receiveBluetoothMessages
//===================================================

//===================================================
//CALIBRATION CONFIGURATION BETWEEN PROGRAMS
//---------------------------------------------------
//#define calibBetweenGames
//===================================================

//===================================================
//DEFINITIONS TO SELECT THE MOVEMENT TYPE
//---------------------------------------------------
//#define funcMoveTo
#define funcAccelRamp
//#define accelRampDebug
//===================================================


//#define relacionMicroSteps1 
//#define relacionMicroSteps4
//#define relacionMicroSteps16
//#define relacionMicroSteps32
//#define relacionMicroSteps64
#define relacionMicroSteps128

//#define originalSpeed
#define halfSpeed


//===================================================
//DEFINITIONS TO CONFIGURE GLOBAL SPEED AND ACCELERATION
//---------------------------------------------------
#ifdef relacionMicroSteps1
#define defGlobalSpeed     400     //Works 500       //400
#define defGlobalAccel     800     //Works 1000      //800
#endif

#ifdef relacionMicroSteps4
#define defGlobalSpeed     6000     //Previously 8000
#define defGlobalAccel     12000     //Previously 16000
#endif

#ifdef relacionMicroSteps16
#define defGlobalSpeed     9000     //Original 12000
#define defGlobalAccel     18000     //Original 24000
#endif

#ifdef relacionMicroSteps32
#define defGlobalSpeed     2000     //Previously 4000
#define defGlobalAccel     8000     //Previously 5000
#endif

#ifdef relacionMicroSteps64
#define defGlobalSpeed     25000     //
#define defGlobalAccel     50000     //
#endif

#ifdef relacionMicroSteps128
#ifdef originalSpeed
    #define defGlobalSpeed     50000     //
    #define defGlobalAccel     100000    //
#endif

#ifdef halfSpeed
    #define defGlobalSpeed     25000     //
    #define defGlobalAccel     50000    //
#endif

#endif
//===================================================


//===================================================
//SENSOR MATRIX ROTATION CONFIGURATION
//---------------------------------------------------
#define matrizPos1 1    // (1) Matrix rotated 180 degrees clockwise
#define matrizPos2 2    // (2) Matrix rotated 90 degrees clockwise
#define matrizPos3 3    // (3) Original matrix
#define matrizPos4 4    // (4) Matrix rotated 270 degrees clockwise

#ifdef CambiosPCBTavo
#define PosMatrizSensor matrizPos2    //matrizPos4  
#endif

#ifdef CambiosPCBLalo
#define PosMatrizSensor matrizPos2  
#endif
//===================================================


//===================================================
//SENSOR TYPE CONFIGURATION
//---------------------------------------------------
#define newSensors
//#define oldSensors
//===================================================


//===================================================
//CONFIGURATION FOR THE PIECE-FREE TRAVERSE
//---------------------------------------------------
#define moveDirectPosPiece                  //Moves toward the target piece in a direct trajectory

//.....Speed in piece-free trajectory using moveToPoint.....
#ifdef relacionMicroSteps1
#define maximumVelocityDirect       400    //Works 500     //400

//.....Speed in piece-free trajectory using accelRamp
#define maximunVelocityDirectRamp   400    //Works 500     //400
#define maximunAccelDirectRamp      800    //Works 1000    //800
#endif

#ifdef relacionMicroSteps4
#define maximumVelocityDirect       6000    //Previously 5000    Defines the speed when moving in a direct trajectory

//.....Speed in piece-free trajectory using accelRamp
#define maximunVelocityDirectRamp   6000    //Previously 4000
#define maximunAccelDirectRamp      12000
#endif

#ifdef relacionMicroSteps16
#define maximumVelocityDirect       9000    //Original 12000    Defines the speed when moving in a direct trajectory

//.....Speed in piece-free trajectory using accelRamp
#define maximunVelocityDirectRamp   9000    //Original 12000
#define maximunAccelDirectRamp      18000    //Original 24000
#endif

#ifdef relacionMicroSteps32
#define maximumVelocityDirect       2000    //Previously 5000    Defines the speed when moving in a direct trajectory

//.....Speed in piece-free trajectory using accelRamp
#define maximunVelocityDirectRamp   2000    //Previously 4000
#endif


#ifdef relacionMicroSteps64
#define maximumVelocityDirect       20000    //

//.....Speed in piece-free trajectory using accelRamp
#define maximunVelocityDirectRamp   25000    //
#define maximunAccelDirectRamp      50000    //
#endif

#ifdef relacionMicroSteps128
#ifdef originalSpeed
#define maximumVelocityDirect       40000    //
//.....Speed in piece-free trajectory using accelRamp
#define maximunVelocityDirectRamp   50000    //
#define maximunAccelDirectRamp      100000    //
#endif
#ifdef halfSpeed
#define maximumVelocityDirect       20000    //
//.....Speed in piece-free trajectory using accelRamp
#define maximunVelocityDirectRamp   25000    //
#define maximunAccelDirectRamp      50000    //
#endif
#endif
//================================================================

#define desfaseElectroiman 3                //Original 6 Value of the distance where the electromagnet turns off before turning on again to center the pieces
#define centrarElectromagnet                //Enables the action of turning the electromagnet off and on to center the pieces


//================================================================
#define ROBOT_4THREADS       1  
#define ROBOT_H              2  
//---------------------------------------------------------------
#define FOUR_DRIVERS         1
#define TWO_DRIVERS          2
//---------------------------------------------------------------
#define MACHINE_STYLE ROBOT_H       //Change this
#define BOARD_CHESS TWO_DRIVERS     //Change this
//================================================================

#include "MechanismH.h"

#if BOARD_CHESS == FOUR_DRIVERS
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (14)
#define MOTOR_0_STEP_PIN (25)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (32)   //Previously 33
#define MOTOR_1_STEP_PIN (33) //Previously 32
//================MOTOR 3====================
#define MOTOR_2_DIR_PIN (18)   
#define MOTOR_2_STEP_PIN (13)  
//================MOTOR 4====================
#define MOTOR_3_DIR_PIN (21)     //Previously 19
#define MOTOR_3_STEP_PIN (19)    //Previously 21

#define MOTOR_0_ENABLE_PIN                  17
#define MOTOR_1_ENABLE_PIN                  17
#define ENABLE_PIN 17

#define hall1 39
#define hall2 36

//TEMPORARY DEFINITIONS FOR TESTING
#define LED_PIN     23
#define HALL3       35
//=====================================

//==================== I2C =================

//#define SDA_PIN 22
//#define SCL_PIN 23
#define I2C_SLAVE_ADDR 0x05
#define I2C_ESP32_ADDR 0x04
#define I2C_ARDUINO_ADDR 0x06

// the maximum is 124
#define MAX_SLAVE_RESPONSE_LENGTH 32

//It is the resistance value placed on the drivers, for example 0.11f represents 0.11 ohms.
#define R_SENSE 0.11f

//STALL_VALUE and STALL_VALUE2 is the sensitivity for collision detection, which also depends on the current.
#define STALL_VALUE 20 //20
#define STALL_VALUE2 20 //20
#define STALL_VALUE3 20 //20
#define STALL_VALUE4 20 //20

//MICROSTEPPING is used to configure the different microstepping settings of the motor
// among the options are 1,2,4,8,16,32,64.
#define MICROSTEPPING 16

#define NORMAL_CURRENT 1200 //800
#define CURRENT_IN_CALIBRATION 1200 //800

#define SERIAL_PORT2 Serial1 // TMC2208/TMC2224 HardwareSerial port

#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b01 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b11 // TMC2209 Driver address according to MS1 and MS2


//Definitions to read from Serial
#define BAUD                                115200
#define STRING_SIZE                         128
#define XOFF                                0x13
#define XON                                 0x11

//Definitions for Steppers and Mechanics
#define pulleyRadius                        6.283185307180
#define neededStepsfor1Turn                 3200
#define maximumVelocitySteppsperSeconds     7000     //Previously 7000
#define MyPI                                3.1415926535897932384626433832795

#endif


#if BOARD_CHESS == TWO_DRIVERS

#ifdef pinoutv1
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (14)
#define MOTOR_0_STEP_PIN (25)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (33) 
#define MOTOR_1_STEP_PIN (32)
//================MOTOR 3====================
#define MOTOR_2_DIR_PIN (0) 
#define MOTOR_2_STEP_PIN (0)  
//================MOTOR 4====================
#define MOTOR_3_DIR_PIN (0)     //Previously 19
#define MOTOR_3_STEP_PIN (0)    //Previously 21

#define MOTOR_0_ENABLE_PIN                  18
#define MOTOR_1_ENABLE_PIN                  18
#define ENABLE_PIN 18

#define hall1 39
#define hall2 36
#define HALL3       13
#define LED_PIN     13

#define ELECTRO_ENABLE_PIN 0
#define BUZZER_PIN 5

#endif

#ifdef pinoutv2
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (33)
#define MOTOR_0_STEP_PIN (32)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (14) 
#define MOTOR_1_STEP_PIN (25)

#define ENABLE_PIN 18
#define ELECTRO_ENABLE_PIN 18

#define LED_PIN     0

#define BUZZER_PIN 5

#endif

#ifdef pinoutv3
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (32)
#define MOTOR_0_STEP_PIN (33)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (17) 
#define MOTOR_1_STEP_PIN (5)

#define ENABLE_PIN 18
#define ELECTRO_ENABLE_PIN 18

#define LED_PIN     0

#define BUZZER_PIN 25

#endif


//=====================================

//==================== I2C =================

//#define SDA_PIN 22
//#define SCL_PIN 23
//#define I2C_SLAVE_ADDR 0x05
//#define I2C_ESP32_ADDR 0x04
//#define I2C_ARDUINO_ADDR 0x06

// the maximum is 124
//#define MAX_SLAVE_RESPONSE_LENGTH 32

#define EEPROM_SIZE 512

//It is the resistance value placed on the drivers, for example 0.11f represents 0.11 ohms.

#ifdef pinoutv1 
#define R_SENSE 0.11f
#endif

#ifdef pinoutv2 
#define R_SENSE 0.10f
#endif

#ifdef pinoutv3 
#define R_SENSE 0.10f
#endif

//STALL_VALUE and STALL_VALUE2 is the sensitivity for collision detection, which also depends on the current.
#define STALL_VALUE 20 //20
#define STALL_VALUE2 20 //20
#define STALL_VALUE3 20 //20
#define STALL_VALUE4 20 //20

//MICROSTEPPING is used to configure the different microstepping settings of the motor
// among the options are 8,16,32,64.
#ifdef relacionMicroSteps1
#define MICROSTEPPING 0  
#endif

#ifdef relacionMicroSteps4
#define MICROSTEPPING 4  
#endif

#ifdef relacionMicroSteps16
#define MICROSTEPPING 16
#endif

#ifdef relacionMicroSteps32
#define MICROSTEPPING 32
#endif

#ifdef relacionMicroSteps64
#define MICROSTEPPING 64
#endif

#ifdef relacionMicroSteps128
#define MICROSTEPPING 128
#endif


#ifdef pinoutv1
#define NORMAL_CURRENT 1200 //1200
#define CURRENT_IN_CALIBRATION 1200 //1200
#endif

#ifdef pinoutv2
#define NORMAL_CURRENT 500 //1200
#define CURRENT_IN_CALIBRATION 500 //1200
#endif

#ifdef pinoutv3
#define NORMAL_CURRENT 500 //900
#define CURRENT_IN_CALIBRATION 500 //900
#endif


#define I_HOLD 1 //12

#define SERIAL_PORT2 Serial1 // TMC2208/TMC2224 HardwareSerial port

#ifdef pinoutv1
#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b11 // TMC2209 Driver address according to MS1 and MS2
#endif


#ifdef pinoutv2
#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b11 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b01 // TMC2209 Driver address according to MS1 and MS2

#endif

#ifdef pinoutv3
#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b11 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b01 // TMC2209 Driver address according to MS1 and MS2

#endif

//Definitions to read from Serial
#define BAUD                                115200
#define STRING_SIZE                         128
#define XOFF                                0x13
#define XON                                 0x11

//Definitions for Steppers and Mechanics
#define pulleyRadius                        6.283185307180

#ifdef relacionMicroSteps1
#define neededStepsfor1Turn                 200     //3200
#endif

#ifdef relacionMicroSteps4
#define neededStepsfor1Turn                 800     //3200
#endif

#ifdef relacionMicroSteps16
#define neededStepsfor1Turn                 3200     //3200
#endif

#ifdef relacionMicroSteps32
#define neededStepsfor1Turn                 6400     //3200
#endif

#ifdef relacionMicroSteps64
#define neededStepsfor1Turn                 12800     //3200
#endif

#ifdef relacionMicroSteps128
#define neededStepsfor1Turn                 25600     //3200
#endif

#ifdef relacionMicroSteps1
#define maximumVelocitySteppsperSeconds     800     //Works 1000 //800
#endif

#ifdef relacionMicroSteps4
#define maximumVelocitySteppsperSeconds     6000     //Previously 7000     9800
#endif

#ifdef relacionMicroSteps16
#define maximumVelocitySteppsperSeconds     9000     //Original 12000
#endif

#ifdef relacionMicroSteps32
#define maximumVelocitySteppsperSeconds     2000     //Previously 7000     9800
#endif

#ifdef relacionMicroSteps64
#define maximumVelocitySteppsperSeconds     25000     //Previously 7000     9800
#endif

#ifdef relacionMicroSteps128
#ifdef originalSpeed
#define maximumVelocitySteppsperSeconds     50000     //Previously 7000     9800
#endif
#ifdef halfSpeed
#define maximumVelocitySteppsperSeconds     25000     //Previously 7000     9800
#endif
#endif

#define MyPI                                3.1415926535897932384626433832795


//The line below makes the compiler to copy the config.h once, even is it's called multiples times. 
// #pragma one
/////////////////////////////////////////////////////////////
//Time between sensors to allow gate switching in ms
#define timeBsensors 100
#define CONSTOFREJECTION 1000
#define SIMPLE 0
#define FULL 1
#define GROUPOFSENSORS 2
#define REJECTION 3

#ifdef pinoutv1
 //Pins below ares used to select the 8bits muxes 
 #define mux8_0 17
 #define mux8_1 5
 #define mux8_2 16
 //Pins for the 4 selectors of the 16bits muxes. 
 #define mux16_0 4
 #define mux16_1 2
 #define mux16_2 15

//Data IN pins from 16 bits muxes
 #define mux16Out_1 35
 #define mux16Out_2 34
 #define mux16Out_3 21
 #define mux16Out_4 19
 #endif

#ifdef pinoutv2
 //Pins below ares used to select the 8bits muxes 
 #define mux8_0 17
 #define mux8_1 2
 #define mux8_2 16
 //Pins for the 4 selectors of the 16bits muxes. 
 #define mux16_0 5
 #define mux16_1 19
 #define mux16_2 21

//Data IN pins from 16 bits muxes
 #define mux16Out_1 34
 #define mux16Out_2 35
 #define mux16Out_3 15
 #define mux16Out_4 4
#endif


#ifdef pinoutv3
 //Pins below ares used to select the 8bits muxes
 #define mux8_0 12//12 4
 #define mux8_1 14//14 13
 #define mux8_2 16//4  2
 //Pins for the 4 selectors of the 16bits muxes.
 #define mux16_0 2//13  25
 #define mux16_1 15//2  14
 #define mux16_2 13//25  12
//Data IN pins from 16 bits muxes
 #define mux16Out_1 35//35  39 
 #define mux16Out_2 34//34  34 
 #define mux16Out_3 39//39  35 
 #define mux16Out_4 36//36  36 
#endif

//To address a sensor, we need to know 3 things: the ESP input it is connected to, the 16-bit mux it is connected to via its 8-bit mux, and finally the address on that 8-bit mux.
//With 8 bits we are able to address all 500 sensors.
//The byte structure is as follows: 0 000 0000 

#endif

#endif // CONFIGURE_H
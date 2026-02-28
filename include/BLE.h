#pragma once
//#define BLUECOMMENTS
#include <Arduino.h>
#include "config.h"



#define SERVICE_UUID5 "fd31a840-22e7-11eb-adc1-0242ac120002" //General service


//====General Config====
#define GENERAL_UUID_TEST_SENSORS               "7b204548-30c3-11eb-adc1-0242ac120002"                  //This characteristic is for sensor testing
#define GENERAL_UUID_ERRORMSG                   "7b204d4a-30c3-11eb-adc1-0242ac120002"

//=============== Definition of the UUIDs of the NewCharacteristic Characteristics (3)
#define GENERAL_UUID_RECEIVE_MOVEMENT           "c60c786b-bf3f-49d8-bd9e-c268e0519a7b"      //Send movement, sent in the following format "M f7-f5", it is the start coordinate and the final coordinate of the movement
#define GENERAL_UUID_STATUS_BOARD               "06034924-77e8-433e-ac4c-27302e5e853f"      //When a piece movement is made on the board, said movement is sent through the following characteristic
#define GENERAL_UUID_SELECT_MODE                "c08d3691-e60f-4467-b2d0-4a4b7c72777e"      //Select mode, sculpture = "1" play = "2"

//OTA Update
#define FILE_UUID_RECEIVE_OTA                   "93601602-bbc2-4e53-95bd-a3ba326bc04b"      //The characteristic for OTA update
//====================================================================================

/**
 * @class Bluetooth
 * @brief Manages Bluetooth communication
 * @param timeOutBt is the time, in milliseconds, to wait for a response from the connected Bluetooth device.
 * @param dataBt is where the received information will be stored.
 */
class Bluetooth {
    private:
        
    public:
        Bluetooth(); 
        int init(String = "Chess");

        static void setPlaylistName(String );
        static void setPathAmount(int);
        static void setPathName(String);
        static void setAddPathNamePlaylist(String);
        static void setPathNameNotify(String);
        static void setPathPosition(int);
        static void setPlayMode(int);
        static void setPathProgress(int);
        static void setPlaylistNameFiles(String );
        static void setPlaylistToggle();

        static void setLedSpeed(int);
        static void setCycleMode(int);
        static void setLedDirection(int);
        static void setBrightness(uint16_t);
        static void setIndexPalette(int);
        static void setRed();
        static void setGreen();
        static void setBlue();
        static void setPositions();
        static void setAmountOfColors();

        static void setVersion(String);
        static void setSimpleVersion(String);
        static void setName(String);
        static void setNotifySensorChange(String);

        //===================Funciones para bluetooth Chess==================
        static String verifNewCommandBluetooth();
        static void setStatus(String);
        static String getModeChess();
        //====================================================================



        static void setTimePaths(String);
        static void setTimeOffOn(String);


        static void setMotorSpeed(int);
        static void setPercentage(int);
        static void setCalibrationStatus(bool);

        static void setTestResult(String);
};
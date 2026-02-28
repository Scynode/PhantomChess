#include <CalibrationXY.h>
#if MACHINE_STYLE == ROBOT_H

hw_timer_t *timer1H = NULL;
int flag_timerH = 0;
int stall_outH = 0;
int *Ap_stallH = &stall_outH;
volatile int contPasosInterrup = 0;

#ifdef relacionMicroSteps1
int velInterruptions =  1250;    //For a full step
#endif

#ifdef relacionMicroSteps4
int velInterruptions =  5000;
#endif

#ifdef relacionMicroSteps16
int velInterruptions =  2000;
#endif

#ifdef relacionMicroSteps32
int velInterruptions =  4000;
#endif

#ifdef relacionMicroSteps64
int velInterruptions =  8000;   //For 1/64 step
#endif


#ifdef relacionMicroSteps128
int velInterruptions =  16000;   //For 1/128 step
#endif

TMC2209Stepper driver(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS2);

extern AccelStepper stepper1;
extern AccelStepper stepper2;
extern MultiStepper steppers;

void normal_turnH();
void check_stallH(int, int *);
void IRAM_ATTR onTimerH();
void testMotorsH(void);
void configDriversH(void);

void calibraEjeY(void);
void calibraEjeX(void);

void calibraEjeXv2(void);
void calibraEjeYv2(void);


void move(int, int);

CalibrationXY::CalibrationXY()
{
}

/**================================================================================================
 *                                           Funcion Calibration::init()
 *  
 *  Performs the necessary configurations for using stepper motors, as well as the initial
 *  configurations for digital pins.
 * 
 *  Also initializes the normal_turnH() function for motor movement via interrupts.
 *================================================================================================**/

void CalibrationXY::initCalibration()
{
	SERIAL_PORT2.begin(115200);
	
        //===========Motor 1 y 2============
	pinMode(MOTOR_0_STEP_PIN, OUTPUT);
	pinMode(MOTOR_0_DIR_PIN, OUTPUT);
        pinMode(MOTOR_1_STEP_PIN, OUTPUT);
	pinMode(MOTOR_1_DIR_PIN, OUTPUT);
        digitalWrite(MOTOR_0_DIR_PIN, LOW);
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
	
        configDriversH();

        digitalWrite(MOTOR_0_DIR_PIN, HIGH);
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        pinMode(ENABLE_PIN, OUTPUT);
        #ifdef CambiosPCBTavo
        digitalWrite(ENABLE_PIN, LOW);
        #endif
        #ifdef CambiosPCBLalo
        digitalWrite(ENABLE_PIN, HIGH);
        #endif
        testMotorsH();
        driver.rms_current(CURRENT_IN_CALIBRATION);
        driver.microsteps(MICROSTEPPING);
        delay(100);

        driver2.rms_current(CURRENT_IN_CALIBRATION);
        driver2.microsteps(MICROSTEPPING);
        delay(100);

        flag_timerH = 0;                                 //flag_timerH controls the interrupts for motor movement
        normal_turnH();

        #ifdef tempTest
        flag_timerH = 3;   //
        #endif

}
/*================================================================================================*/


/**================================================================================================
 *                                           Funcion Calibration::start()
 *  
 *  This is the main function of the calibration process for the H-mechanism.
 *  Two calibration sequences are performed, one on the x axis and one on the y axis to find
 *  the center of the system
 *================================================================================================**/
int CalibrationXY::startCalibration()
{       
        driver.en_spreadCycle(false);
        driver2.en_spreadCycle(false);

        #ifdef debug
        Serial.println("MicroSteps");
        Serial.println(driver.microsteps());
        Serial.println(driver2.microsteps());
        #endif
        delay(500);

        //calibraEjeY();          //Function for y-axis calibration
        calibraEjeYv2();
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                             //Direction configuration -> Downward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        #endif
        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);     //LOW                           //Direction configuration -> Downward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW);    //HIGH
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);    //HIGH                         //Direction configuration -> Downward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW);     //LOW
        #endif

        #ifdef relacionMicroSteps1
        move(1055,1000);          //For a full step
        #endif

        #ifdef relacionMicroSteps4
        move(4220,500);          //4180 previously delay 200 (18370)    Moves to compensate the mechanical offset of a piece (Temporary)  move(18350,500); 
        #endif

        #ifdef relacionMicroSteps16
        move(16880,100);
        #endif
        
        #ifdef relacionMicroSteps32
        move(36760,100);
        #endif

        #ifdef relacionMicroSteps64
        move(67520,15);          //For 1/64 step
        #endif

        #ifdef relacionMicroSteps128
        move(135040,7);          //For 1/128 step
        #endif

        //calibraEjeX();          //Function for x-axis calibration
        calibraEjeXv2();
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                            //Direction configuration -> Leftward movement on x axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        #endif
        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, LOW); //LOW                           //Direction configuration -> Leftward movement on x axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW); //LOW
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, LOW);  //LOW                          //Direction configuration -> Leftward movement on x axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW);  //LOW
        #endif

        #ifdef relacionMicroSteps1
        #ifdef CambiosPCBTavo
        move(1010,1000);          //For full steps 
        #endif
        #ifdef CambiosPCBLalo
        move(999,1000);          //For full steps     
        #endif
        #endif
        
        #ifdef relacionMicroSteps4
        #ifdef CambiosPCBTavo
        move(4040,500);          //3930 previously delay 200 (18450)    Moves to compensate the gondola width (Temporary) move(18350,500);      
        #endif
        #ifdef CambiosPCBLalo
        move(3996,500);          //3930 previously delay 200 (18450)    Moves to compensate the gondola width (Temporary) move(18350,500);      
        #endif
        #endif

        #ifdef relacionMicroSteps16
        #ifdef CambiosPCBTavo
        move(16160,100);
        #endif
        #ifdef CambiosPCBLalo
        move(15984,100);
        #endif
        #endif

        #ifdef relacionMicroSteps32
        #ifdef CambiosPCBTavo
        move(32320,100);
        #endif
        #ifdef CambiosPCBLalo
        move(31968,100);
        #endif
        #endif

        #ifdef relacionMicroSteps64
        #ifdef CambiosPCBTavo
        move(64640,15);          //For 1/64 step    
        #endif
        #ifdef CambiosPCBLalo   
        move(63936,15);          //For 1/64 step    
        #endif
        #endif

        #ifdef relacionMicroSteps128
        #ifdef CambiosPCBTavo
        move(129280,7);          //For 1/128 step    
        #endif
        #ifdef CambiosPCBLalo   
        move(127872,7);          //For 1/128 step    
        #endif
        #endif

        driver.en_spreadCycle(false);    //true
        driver2.en_spreadCycle(false);   //true
        
        stepper1.setCurrentPosition(0);                           
        stepper2.setCurrentPosition(0);
        return 0;
}
/*================================================================================================*/

/**
 * @brief This function allows moving the motors in a controlled manner by step count. 
 * @param pasos This variable indicates the number of steps to advance.
 * @param Speed Indicates the rotation speed.
 */
void move(int pasos, int Speed)
{
        for (int i = 0; i < pasos; i++)
        {
                digitalWrite(MOTOR_0_STEP_PIN, LOW);
                delayMicroseconds(Speed);
                digitalWrite(MOTOR_0_STEP_PIN, HIGH);
                delayMicroseconds(Speed);

                digitalWrite(MOTOR_1_STEP_PIN, LOW);
                delayMicroseconds(Speed);
                digitalWrite(MOTOR_1_STEP_PIN, HIGH);
                delayMicroseconds(Speed);
        }
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion check_stallH
 *?  Verifies the current value of StallGuard and returns "1" if StallGuard has been triggered
 *?  and returns 0 otherwise
 *@param result_stall           Returns the StallGuard result  
 *@param flag_timerH             Stops the interrupts when a state change has been detected   
 *@return void
 *================================================================================================**/
void check_stallH(int driver_m, int *result_stall)
{
	int stall_data = 0;
	if (driver_m == 1)
	{
                stall_data = driver.SG_RESULT();                //Requests the StallGuard value from the driver
                #ifdef relacionMicroSteps1
                #ifdef CambiosPCBTavo
		if (stall_data < 250)   //For full steps
                #endif
                #ifdef CambiosPCBLalo
		if (stall_data < 200)   //For full steps
                #endif
                #endif 
                #ifdef relacionMicroSteps4
		if (stall_data < 125)   //120
                #endif                            //Lower value = lower sensitivity
                #ifdef relacionMicroSteps16
                if (stall_data < 140)
                #endif
                #ifdef relacionMicroSteps32
                if (stall_data < 15)
                #endif
                #ifdef relacionMicroSteps64
                #ifdef CambiosPCBTavo
                if (stall_data < 110)     //For 1/64 step
                #endif
                #ifdef CambiosPCBLalo
                if (stall_data < 100)     //For 1/64 step
                #endif
                #endif 
                #ifdef relacionMicroSteps128
                #ifdef CambiosPCBTavo
                if (stall_data < 110)     //For 1/128 step
                #endif
                #ifdef CambiosPCBLalo
                if (stall_data < 100)     //For 1/128 step
                #endif
                #endif
        	{
			flag_timerH = 0;
                        #ifdef debug
			Serial.println("Se activo stall guard M1");
                        #endif
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}

	if (driver_m == 2)
	{
                stall_data = driver2.SG_RESULT();                //Requests the StallGuard value from the driver


                #ifdef relacionMicroSteps1
		if (stall_data < 250)       //For full steps
                #endif   
		#ifdef relacionMicroSteps4
		if (stall_data < 125)       //120
                #endif                            //Lower value = lower sensitivity    Previously 25   //Previously 35
                #ifdef relacionMicroSteps16
                if (stall_data < 140)
                #endif
                #ifdef relacionMicroSteps32
                if (stall_data < 15)
                #endif
                #ifdef relacionMicroSteps64
                if (stall_data < 110)         //For 1/64 step
                #endif 
                #ifdef relacionMicroSteps128
                if (stall_data < 110)         //For 1/128 step
                #endif
		{
			flag_timerH = 0;
                        #ifdef debug
			Serial.println("Se activo stall guard M2");
                        #endif
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion testMotorsH()
 *?  This function performs current and microstep configurations, and verifies that the driver
 *?  has been configured with the correct values
 *@param name 
 *@param name  
 *@return void
 *================================================================================================**/
void testMotorsH()
{
    /**==============================================
     *                Test Motor 1
     *=============================================**/

    /*======= Prueba comunicacion con el driver =======*/
    #ifdef debug
    Serial.println(" ");
    Serial.println("Test Motor 1");
    #endif
    uint8_t result_1 = driver.test_connection();
	if (result_1 == 0)
	{
                #ifdef debug
		Serial.println(F("M1_OK,"));
                #endif
	}
	else
	{
                #ifdef debug
		Serial.println(F("M1_Fail,"));
                #endif
	}
       Serial.println(" "); 
    /*======= Prueba configuracion a 700mA y 32 microsteps =======*/   
    Serial.println("M1 configuracion a 700mA y 32 microsteps"); 
    driver.rms_current(700);
    driver.microsteps(32);
    #ifdef debug
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 900mA y 64 microsteps =======*/
    Serial.println("M1 configuracion a 900mA y 64 microsteps");
    driver.rms_current(900);
    driver.microsteps(64);

    #ifdef debug
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 500mA y 0 microsteps =======*/
    Serial.println("M1 configuracion a 500mA y 0 microsteps");
    driver.rms_current(CURRENT_IN_CALIBRATION);
    driver.microsteps(MICROSTEPPING);
    #ifdef debug
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    #endif
    /*=============================================*/
    
    /**==============================================
     *                Test Motor 2
     *=============================================**/
Serial.println(" ");
    /*======= Prueba comunicacion con el driver =======*/
    #ifdef debug
    Serial.println("Test Motor 2");
    #endif
    uint8_t result_2 = driver2.test_connection();
	if (result_2 == 0)
	{
                #ifdef debug
		Serial.println(F("M2_OK,"));
                #endif
	}
	else
	{
                #ifdef debug
		Serial.println(F("M2_Fail,"));
                #endif
	}
        Serial.println(" ");
    /*======= Prueba configuracion a 700mA y 32 microsteps =======*/ 
    Serial.println("M2 configuracion a 700mA y 32 microsteps");
    driver2.rms_current(700);
    driver2.microsteps(32);
    #ifdef debug
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 900mA y 64 microsteps =======*/
    Serial.println("M2 configuracion a 900mA y 64 microsteps");
    driver2.rms_current(900);
    driver2.microsteps(64);
    #ifdef debug
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 500mA y 0 microsteps =======*/
    Serial.println("M2 configuracion a 500mA y 0 microsteps");
    driver2.rms_current(CURRENT_IN_CALIBRATION);
    driver2.microsteps(MICROSTEPPING);
    #ifdef debug
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    #endif
    /*=============================================*/
    /*
        Serial.println(" ");
    Serial.println("Test de Buzzer");
    cute.init(BUZZER_PIN);
    ledcSetup(5, 2000, 16);   // channel, max frequency, resolution
    ledcAttachPin(BUZZER_PIN, 5);

    cute._tone(NOTE_C6,50,30); /// 1046.5  //C6
delay(500);
cute._tone(NOTE_Db6,50,30); /// 1108.73  //C#6/Db6
delay(500);
cute._tone(NOTE_D6,50,30); /// 1174.66  //D6
delay(500);
cute._tone(NOTE_Eb6,50,30); /// 1244.51  //D#6/Eb6
delay(500);
cute._tone(NOTE_E6,50,30); /// 1318.51  //E6
delay(500);
cute._tone(NOTE_F6,50,30); /// 1396.91  //F6
delay(500);
cute._tone(NOTE_Gb6,50,30); /// 1479.98  //F#6/Gb6
delay(500);
cute._tone(NOTE_G6,50,30); /// 1567.98  //G6
delay(500);
cute._tone(NOTE_Ab6,50,30); /// 1661.22  //G#6/Ab6
delay(500);
cute._tone(NOTE_A6,50,30); /// 1760  //A6
delay(500);
cute._tone(NOTE_Bb6,50,30); /// 1864.66  //A#6/Bb6
delay(500);
cute._tone(NOTE_B6,50,30); /// 1975.53  //B6
delay(500);


cute._tone(NOTE_C7,50,30); /// 2093  //C7
delay(500);
cute._tone(NOTE_Db7,50,30); /// 2217.46  //C#7/Db7
delay(500);
cute._tone(NOTE_D7,50,30); /// 2349.32  //D7
delay(500);
cute._tone(NOTE_Eb7,50,30); /// 2489.02  //D#7/Eb7
delay(500);
cute._tone(NOTE_E7,50,30); /// 2637.02  //E7
delay(500);
cute._tone(NOTE_F7,50,30); /// 2793.83  //F7
delay(500);
cute._tone(NOTE_Gb7,50,30); /// 2959.96  //F#7/Gb7
delay(500);
cute._tone(NOTE_G7,50,30); /// 3135.96  //G7
delay(500);
cute._tone(NOTE_Ab7,50,30); /// 3322.44  //G#7/Ab7
delay(500);
cute._tone(NOTE_A7,50,30); /// 3520  //A7
delay(500);
cute._tone(NOTE_Bb7,50,30); /// 3729.31  //A#7/Bb7
delay(500);
cute._tone(NOTE_B7,50,30); /// 3951.07  //B7
delay(500);
Serial.println(" ");
*/

}
/**================================================================================================
 **                                      Funcion normal_turnH()
 *?  In this function the necessary timer and counter configurations are made to generate
 *?  the interrupts that will control motor movement
 *@return void
 *================================================================================================**/
void normal_turnH()
{

        cli();                                        //stop interrupts
        timer1H = timerBegin(3, 8, true);              // Configures the timer; here using timer 4 of the 4 available in the ESP (0,1,2,3)
                                                      // the prescaler is 8, and true is a flag indicating whether the interrupt fires on edge or level
        timerAttachInterrupt(timer1H, &onTimerH, true); //Links the timer with the AttachInterrupt function
                                                      //which is executed when the interrupt is generated
        timerAlarmWrite(timer1H, velInterruptions, true);          //This function defines the counter value at which the timer interrupt is generated, original 10000
        timerAlarmEnable(timer1H);                     //Function to enable the timer.
        sei();                                        //allow interrupts
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion IRAM_ATTR onTimerH()
 *?  In this function the STEP pins are activated each time an interrupt occurs.
 *?  The flag_timerH flag helps indicate which motors receive the control signal.
 *@return void
 *================================================================================================**/
void IRAM_ATTR onTimerH()
{
	if (flag_timerH == 1)                                                            //Controls the rotation of motor 1
	{
		digitalWrite(MOTOR_0_STEP_PIN, !digitalRead(MOTOR_0_STEP_PIN));
	}
        if (flag_timerH == 2)                                                            //Controls the rotation of motor 2
	{
		digitalWrite(MOTOR_1_STEP_PIN, !digitalRead(MOTOR_1_STEP_PIN));
	}
	if (flag_timerH == 3)                                                            //Controls the rotation of both motors
	{
		digitalWrite(MOTOR_1_STEP_PIN, !digitalRead(MOTOR_1_STEP_PIN));
                digitalWrite(MOTOR_0_STEP_PIN, !digitalRead(MOTOR_0_STEP_PIN));
                contPasosInterrup++;
	}
}
/*================================================================================================*/

/**================================================================================================
 **                                      configDriversH()
 *?  In this function the necessary driver configurations are performed
 *@return void
 *================================================================================================**/
void configDriversH(void)
{
	//===========Motor 1 y 2============
	driver.begin();

	driver.pdn_disable(true); // enables the  PDN/UART comunication.

	driver.toff(4); // Sets the slow decay time (off time) [1 ... 15]
					// This setting also limits the maximum chopper frequency. To operate with StealthChop
					// In case of operating only with StealthChop, any setting is fine.

	driver.blank_time(24);

	driver.rms_current(CURRENT_IN_CALIBRATION); // Sets the current value

	driver.microsteps(MICROSTEPPING); // Sets the microstep value

	driver.TCOOLTHRS(0xFFFFF); // Lower speed threshold for enabling intelligent CoolStep and StallGuard on the DIAG output

	driver.semin(0); // Lower CoolStep threshold [0 ... 15].
					 // If SG_RESULT falls below this threshold, CoolStep increases the current to both coils.
					 // 0: disable CoolStep

	driver.shaft(false); //Sets the motor rotation direction via UART communication

	driver.sedn(0b01); // Sets the number of StallGuard2 readings above the upper threshold required
					   // for each current decrease of the motor current.

	driver.SGTHRS(STALL_VALUE); // StallGuard4 threshold level [0 ... 255] for stall detection. Compensates
								// motor-specific characteristics and controls sensitivity. A higher value gives a higher
								// sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
								// indicate opposition to movement.

	driver2.begin();

	driver2.pdn_disable(true); // Enables PDN/UART communication

	driver2.toff(4); // Sets the slow decay time (off time) [1 ... 15]
					 // This setting also limits the maximum chopper frequency. To operate with StealthChop
					 // In case of operating only with StealthChop, any setting is fine.

	driver2.blank_time(24);

	driver2.rms_current(CURRENT_IN_CALIBRATION); // Sets the current value

	driver2.microsteps(MICROSTEPPING); // Sets the microstep value

	driver2.TCOOLTHRS(0xFFFFF); // Lower speed threshold for enabling intelligent CoolStep and StallGuard on the DIAG output

	driver2.semin(0); // Lower CoolStep threshold [0 ... 15].
					  // If SG_RESULT falls below this threshold, CoolStep increases the current to both coils.
					  // 0: disable CoolStep

	//driver.semax(2);

	driver2.shaft(false); // Sets the motor rotation direction via UART communication

	driver2.sedn(0b01); // Sets the number of StallGuard2 readings above the upper threshold required
						// for each current decrease of the motor current.

	driver2.SGTHRS(STALL_VALUE2); // StallGuard4 threshold level [0 ... 255] for stall detection. Compensates
								  // motor-specific characteristics and controls sensitivity. A higher value gives a higher
								  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
								  // indicate opposition to movement.

	driver.ihold(I_HOLD);
	driver2.ihold(I_HOLD);

	
	//EEPROM.begin(EEPROM_SIZE);
	//delay(1000);
	int cont = 0;
	while (driver.microsteps() != MICROSTEPPING)
	{
		driver.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}
	cont = 0;
	while (driver2.microsteps() != MICROSTEPPING)
	{
		driver2.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}
        EEPROM.begin(EEPROM_SIZE);

        //Erase data in EEPROM
        float val;
        val = 0.0;
        EEPROM.writeFloat(401, val);
        EEPROM.commit();

        EEPROM.writeFloat(405, val);
        EEPROM.commit();
        //======================================================================================
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion calibraEjeY()
 *?  Performs a calibration routine that consists of finding the limits of the mechanism on the
 *?  Y axis with the help of StallGuard, then positions itself at one of the extremes and moves toward the
 *? central position of the axis. 
 *@param name type    
 *@return void
 *================================================================================================**/
void calibraEjeY()
{
        int sumaInterrup = 0;
        /*================== Primer recorrido hacia la derecha =================*/

        digitalWrite(MOTOR_0_DIR_PIN, LOW);                            //Direction configuration -> Upward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        stall_outH = 0;
        flag_timerH = 0;                                                 //Deactivates interrupts for both motors
        contPasosInterrup = 0;
        delay(500);
        /*================== Second traverse to the left =================*/
/*
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Direction configuration -> Downward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        contPasosInterrup = 0;
        while (stall_outH == 0)
        {
                Serial.println(driver.SG_RESULT(), DEC);
                check_stallH(2, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        sumaInterrup = contPasosInterrup;

        stall_outH = 0;
        flag_timerH = 0;                                                 //Deactivates interrupts for both motors
        contPasosInterrup = 0;
        delay(500);

        sumaInterrup = sumaInterrup / 2;                                //Gets half the interrupt count for the next movement
        Serial.println("Pasos de regreso");
        Serial.println(sumaInterrup);
        delay(500);
  */      
        /*================== Third traverse to the right =================*/
 /*       digitalWrite(MOTOR_0_DIR_PIN, HIGH);                            //Direction configuration -> Upward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        
        flag_timerH = 3;                                                 //Activates interrupts for both motors
        contPasosInterrup = 0;
        while (contPasosInterrup < sumaInterrup)                        //Advances until the interrupt count is complete to reach the center
        {
                Serial.println(contPasosInterrup);
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);
        */
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion calibraEjeX()
 *?  Performs a calibration routine that consists of finding the limits of the mechanism on the
 *?  x axis with the help of StallGuard, then positions itself at one of the extremes and moves toward the
 *? central position of the axis. 
 *@param name type    
 *@return void
 *================================================================================================**/
void calibraEjeX()
{
        int sumaInterrup = 0;
        /*================== First traverse upward =================*/
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                             //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        
        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        stall_outH = 0;

        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);
        
        /*================== Second traverse to the left =================*/
 /*       digitalWrite(MOTOR_0_DIR_PIN, HIGH);                            //Direction configuration -> Leftward movement on x axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        contPasosInterrup = 0;
        while (stall_outH == 0)
        {
                Serial.println(driver.SG_RESULT(), DEC);
                check_stallH(1, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        sumaInterrup = contPasosInterrup;

        stall_outH = 0;

        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);

        sumaInterrup = sumaInterrup / 2;                                //Gets half the interrupt count for the next movement
        Serial.println("Pasos de regreso");
        Serial.println(sumaInterrup);
        delay(500);
        */     
        /*================== Second traverse upward =================*/
 /*      digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Direction configuration -> Upward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        flag_timerH = 3;                                                 //Activates interrupts for both motors
        contPasosInterrup = 0;
        while (contPasosInterrup < sumaInterrup)                        //Advances until the interrupt count is complete to reach the center
        {
                Serial.println(contPasosInterrup);
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);

        */
}




void calibraEjeXv2()
{
        int sumaInterrup = 0;
/*
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                             //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        
        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        stall_outH = 0;
        flag_timerH = 0;
        delay(500);
        digitalWrite(MOTOR_0_DIR_PIN, LOW);
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        flag_timerH = 3;
        contPasosInterrup = 0;
        while(contPasosInterrup < 5000)
        {
                flag_timerH = 3;
        }
        flag_timerH = 0;
*/
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        #endif

        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);     //HIGH                        //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);     //HIGH
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);      //HIGH                       //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);      //HIGH
        #endif

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //For full steps
        #endif
        
        #ifdef relacionMicroSteps4
        velInterruptions = 7000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif

        #ifdef relacionMicroSteps64
        velInterruptions = 500;         //For 1/64 step
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;         //For 1/128 step
        #endif

        normal_turnH();
        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver2.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        stall_outH = 0;
        flag_timerH = 0;

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //For full steps
        #endif

        #ifdef relacionMicroSteps4
        velInterruptions = 6000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif


        #ifdef relacionMicroSteps64
        velInterruptions = 500;         //For 1/64 step
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;         //For 1/128 step
        #endif


        contPasosInterrup = 0;
        delay(500);
        

}



void calibraEjeYv2()
{
        int sumaInterrup = 0;
        /*================== First traverse to the right =================*/
/*
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                            //Direction configuration -> Upward movement on y axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        stall_outH = 0;
        flag_timerH = 0;                                                 //Deactivates interrupts for both motors
        

        delay(500);
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        flag_timerH = 3;
        contPasosInterrup = 0;
        while(contPasosInterrup < 5000)
        {
                flag_timerH = 3;
        }
        flag_timerH = 0;
*/
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        #endif

        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, LOW);   //HIGH                          //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);    //LOW
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, LOW);      //LOW                       //Direction configuration -> Rightward movement on X axis
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);     //HIGH
        #endif

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //For full steps
        #endif

        #ifdef relacionMicroSteps4
        velInterruptions = 7000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif

        #ifdef relacionMicroSteps64
        velInterruptions = 500;           //For 1/64 step
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;           //For 1/128 step
        #endif

        normal_turnH();
        flag_timerH = 3;                                                 //Activates interrupts for both motors
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver2.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checks the StallGuard value until an obstacle is detected
        }
        stall_outH = 0;
        flag_timerH = 0;

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //For full steps
        #endif

        #ifdef relacionMicroSteps4
        velInterruptions = 6000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif

        #ifdef relacionMicroSteps64
        velInterruptions = 500;           //For 1/64 step
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;           //For 1/128 step
        #endif

}





#endif  
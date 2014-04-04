#include "WPILib.h"
#include <math.h>
#include <timer.h>
#include <sstream>

// Constants
#define SLOW_TIME 0.5
#define AUTO_SLOW_TIME 3.0;
#define RIGHT_SCALE_FACTOR 1.0;
#define LEFT_SCALE_FACTOR 1.0;
const int numReadings = 20; // Number of distance sensor readings to avg
const int threshold = 300;  // Distance sensor threshold
int readings[numReadings];  // Readings from the analog input
int sensorIndex = 0;        // Index of the current reading
int total = 0;              // Running total
double myTime = 0;          // Timer for automatic ball pickup
double targetTime = 0;

// Flags
bool compressor_enabled = true;
bool buttonStartFlag = true;
bool fireFlag = false;
bool armFireFlag = false;
bool manualModeToggle = false;
bool autoFlag=false;

// Integer globals
int sonicSensorCount = 0;
int pickupStage = 0;
int printCounter = 0;
int armState = 0; // 0=home position, 1=moving towards extended, 2=extended, 3=moving towards home
int shootState = 0;
int autonomousState = 0;
float rollerSpeed = 0.0;
float armSpeed = 0.0;
float shooterSpeed = 0.0;
float currentTime;

class BuiltinDefaultCode : public IterativeRobot {

	// Motor controllers
	Talon *left_1;
	Talon *left_2;
	Talon *right_1;
	Talon *right_2;
	Victor *gobblerRoller;
	Victor *gobblerPosition;
	Victor *shooter;

	// Compressor
	Compressor *compressor;	
	
	// Solenoids
	Solenoid *shifterHi;
	Solenoid *shifterLo;

	// Axis Camera
	AxisCamera *camera;
	
	// Timer for automatic ball pickup
	Timer *ballPickupTimer;
	Timer *autonomousTimer;

	// Limit switches
	DigitalInput *limitSwitchShooter;
	DigitalInput *limitSwitchPickupLower;
	DigitalInput *limitSwitchPickupUpper;
	DigitalInput *ultraSonicSensor;

	// Joystick
	Joystick *gamePadDriver;  // Silver
	Joystick *gamePadShooter; // Red

    // AxisCamera &camera = AxisCamera::GetInstance();	// To use the Axis camera uncomment this line

public:
	BuiltinDefaultCode(void) {
		printf("BuiltinDefaultCode Constructor Started\n");

		left_1  = new Talon(1);
		left_2  = new Talon(2);
		right_1 = new Talon(3);
		right_2 = new Talon(4);

		gobblerRoller = new Victor(5);
		shooter = new Victor(7);
		gobblerPosition = new Victor(8);

		compressor = new Compressor(1,1);

		shifterHi = new Solenoid(1);
		shifterLo = new Solenoid(2);

		gamePadDriver  = new Joystick(1); // Blue (unmarked)
		gamePadShooter = new Joystick(2); // Red

		limitSwitchPickupLower = new DigitalInput(2);
		limitSwitchPickupUpper = new DigitalInput(3);
		limitSwitchShooter     = new DigitalInput(5);
		ultraSonicSensor       = new DigitalInput(4);

		ballPickupTimer = new Timer();
		autonomousTimer = new Timer();


		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		// UINT32 m_priorPacketNumber;	// keep track of the most recent packet number from the DS
		// UINT8 m_dsPacketsReceivedInCurrentSecond; // keep track of the ds packets received in the current second

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		// m_autoPeriodicLoops = 0;
		// m_disabledPeriodicLoops = 0;
		// m_telePeriodicLoops = 0;

		printf("BuiltinDefaultCode Constructor Completed\n");
	}

	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		ShiftLow();
		printf("RobotInit() completed.\n");
	}

	void DisabledInit(void) {
		// int m_disabledPeriodicLoops = 0; // Reset the loop counter for disabled mode
	}

	void AutonomousInit(void) {
		// int m_autoPeriodicLoops = 0; // Reset the loop counter for autonomous mode
		compressor->Start();
		autonomousTimer->Start();
		autonomousTimer->Reset();
	}

	void TeleopInit(void) {
		// int m_telePeriodicLoops = 0; // Reset the loop counter for teleop mode
		// int m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
		for (int thisReading = 0; thisReading < numReadings; thisReading++) readings[thisReading] = 0; 
		compressor->Start();
		compressor_enabled = true;
		ballPickupTimer->Start();
		ballPickupTimer->Reset();
		shooter->SetSpeed(0.0);
		gobblerPosition->SetSpeed(0.0);
		gobblerRoller->SetSpeed(0.0);
		left_1->SetSpeed(0.0);
		left_2->SetSpeed(0.0);
		right_1->SetSpeed(0.0);
		right_2->SetSpeed(0.0);
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {
	}

	void motorControlLeft(float speed) 
	{
		left_1->SetSpeed(speed);
		left_2->SetSpeed(speed);
	}

	void motorControlRight(float speed)
	{
		right_1->SetSpeed(speed);
		right_2->SetSpeed(speed);
	}

	void AutonomousPeriodic(void) {
		if(autoFlag){
			motorControlLeft(0.0);
			motorControlRight(0.0);
		} else {
			motorControlLeft(0.6);
			motorControlRight(-0.6);
			
			if(!ultraSonicSensor->Get() && sonicSensorCount < 3) {
				sonicSensorCount++;
			}
			else if(ultraSonicSensor->Get()) {
				sonicSensorCount = 0;
			}
			if(sonicSensorCount==3)autoFlag=true;
		}
	}

	void ShiftHigh(void) {
		// shifter->Get() return values: false is low gear, true is high gear
		if(!(shifterHi->Get())) {
			shifterHi->Set(true);
			shifterLo->Set(false);
		}
	}

	void ShiftLow(void) {
		// shifter->Get() return values: false is low gear, true is high gear
		if(shifterHi->Get()) {
			shifterHi->Set(false);
			shifterLo->Set(true);
		}
	}
	
	void compressorToggle(bool buttonState) {
		static bool lastButtonState = false;
		if(buttonState && !lastButtonState) {
			if(compressor_enabled) {
				compressor->Stop();
				compressor_enabled = false;
			} else {
				compressor->Start();
				compressor_enabled = true;
			}
		}
		lastButtonState = buttonState;
	}
	
	void manualToggle(bool buttonState) {
		static bool lastButtonState = false;
		if(buttonState && !lastButtonState) {
			manualModeToggle = manualModeToggle ? false : true;
		}
		lastButtonState = buttonState;
	}

	void TeleopPeriodic(void) {
		float leftStick  = -1 * gamePadDriver->GetRawAxis(2);       // Drive system
		float rightStick = gamePadDriver->GetRawAxis(4);            // Drive system
		bool buttonX  = gamePadShooter->GetRawButton(1);            // Shooting
		bool buttonB  = gamePadShooter->GetRawButton(3);            // Roll out (pass)
		bool buttonY  = gamePadShooter->GetRawButton(4);            // Roll in
		bool leftBumper  = gamePadDriver->GetRawButton(5);          // Shifting (Low)
		bool rightBumper = gamePadDriver->GetRawButton(6);          // Shifting (High)
		bool leftTrigger  = gamePadShooter->GetRawButton(7);        // Gobbler position down
		bool rightTrigger = gamePadShooter->GetRawButton(8);        // Gobbler position up
		bool buttonDriverStart = gamePadDriver->GetRawButton(10);   // Compressor
		bool buttonShooterStart = gamePadShooter->GetRawButton(10); // Manual override toggle
		// bool buttonBack = gamePadShooter->GetRawButton(9);
		// bool buttonA  = gamePadShooter->GetRawButton(2);

		// bool limitSwitchExtended = limitSwitchPickupLower->Get();
		// bool limitSwitchRetracted = limitSwitchPickupUpper->Get();
		bool limitSwitchExtended = limitSwitchPickupLower->Get();
		bool limitSwitchRetracted = limitSwitchPickupUpper->Get(); // POSSIBLE ERROR

		// Compressor toggle
		compressorToggle(buttonDriverStart);

		// Shifting
		if(rightBumper && !leftBumper) {
			ShiftHigh();
		}
		else if(leftBumper && !rightBumper) {
			ShiftLow();
		}

		manualToggle(buttonShooterStart);

		if(manualModeToggle) {
			armSpeed = 0.0;
			rollerSpeed = 0.0;
			
			if(leftTrigger && !rightTrigger) {
				armSpeed = 0.6;
			}
			else if(rightTrigger && !leftTrigger) {
				armSpeed = -1.0;
			}
			armState = 0;
		}
		else if (!fireFlag) {
			switch(armState) {
				// arm is in home position
				case 0:
					rollerSpeed = 0.0; //roller speed is 0
					armSpeed = 0.0; //arm speed is 0;
					if(rightTrigger && !leftTrigger) {
						armState = 1;
					}
					break;
					
				// arm is moving to extended position
				case 1:
					armSpeed = -1.0; // move arm down
					rollerSpeed = 0.7; // run roller in
					if(!limitSwitchExtended) {
						armState = 2; // if limit switch hit, set go to case 2
					}
					if(leftTrigger && !rightTrigger) {
						ballPickupTimer->Reset();
						armState = 3;
					} // if left trigger pressed, go to case 3
					
					// if time > some larger amount, stop everything and switch to manual control
					break;
				
				//arm is extended; ready for pickup
				case 2:
					armSpeed = 0.0; // stop arm movement
					rollerSpeed = 0.7; // run roller in
					if(leftTrigger && !rightTrigger) {
						ballPickupTimer->Reset();
						armState = 3; // if left trigger pressed, go to case 3
					}
					break;

				// arm is moving to retracted position
				case 3:
					float time = ballPickupTimer->Get();
					printf("Time: %f", time);
					if(ballPickupTimer->Get() > SLOW_TIME) {
						rollerSpeed = 0.3; // stop arm movement
					}
					else {
						rollerSpeed = 0.7;
					}
					armSpeed = 0.7; // move arm up
					if(!limitSwitchRetracted) {
						armState = 0; // if limit switch hit, set go to case 0
					}
					if(rightTrigger && !leftTrigger) {
						armState = 1; // if right trigger is pressed, go back to case 1
					}
					// if time > some smaller amount, reduce roller speed
					// if time > some larger amount, stop everything and switch to manual control
					break;

				default:
					// something went wrong, print an angry error message
					printf("ERROR, default case hit");
					break;
			}
			
			if(limitSwitchShooter->Get()) {
				shooterSpeed = 1.0;
			}
			else {
				shooterSpeed = 0.0;
			}
		}
		else{  //firing mode
			switch(shootState){
				case 0:  // move arm out of the way
					armSpeed = -1.0; // move arm down
					rollerSpeed = 0.5; // roll roller along ball

					if(!limitSwitchExtended) {
						shootState = 1;
						armSpeed = 0.0; // move arm down
						rollerSpeed = 0.0; // roll roller along ball
					}
				break;
				
				case 1:  // arm is extended; advance shooter to fire
					if(!limitSwitchShooter->Get()) {
						shooterSpeed = 1.0;
					}
					else {
						fireFlag = false;
						shootState = 0;
						armState = 2;
					}
				break;
			}
		}
		
		// manual control of roller
		if(buttonB) rollerSpeed = -1.0;
		if(buttonY) rollerSpeed = 1.0;
		if(buttonX) fireFlag = true;		

		// Print values (rate limited to 1/20)
//		if(printCounter % 20 == 0) {
//			if(!limitSwitchExtended) printf("Lower Limit Hit\n");
//			if(!limitSwitchRetracted) printf("Upper Limit Hit\n");
//		}
//		printCounter++;

		if(leftStick < 0.1 && leftStick > -0.1) {
			leftStick = 0.0;
		}
		if(rightStick < 0.1 && rightStick > -0.1) {
			rightStick = 0.0;
		}

		// Motor speed declarations done at the end to ensure watchdog is continually updated.
		motorControlLeft(leftStick);
		motorControlRight(rightStick);
		gobblerRoller->SetSpeed(rollerSpeed);
		gobblerPosition->SetSpeed(armSpeed);
		shooter->SetSpeed(shooterSpeed);
	}

	void DisabledContinuous(void) {

	}
        
	void AutonomousContinuous(void) {

	}

	void TeleopContinuous(void) {

	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);

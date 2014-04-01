#include "WPILib.h"
#include <math.h>
#include <sstream>

#define TIME_WAIT 0.15      // Time delay before ball pickup
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
bool armPickupFlag = true;
bool timerFlag;
bool pickupFlag = false;
bool abortFlag = false;
bool fireFlag = false;
bool armFireFlag = false;

int pickupStage = 0;
int potentiometerValue;
int printCounter = 0;
float rollerSpeed = 0.0;
float armSpeed = 0.0;
float shooterSpeed = 0.0;

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

	// Limit switches
	DigitalInput *limitSwitchShooter;
	DigitalInput *limitSwitchPickupLower;
	DigitalInput *limitSwitchPickupUpper;

	// Joystick
	Joystick *gamePadDriver;  // Silver
	Joystick *gamePadShooter; // Red

    // Declare angle of the robot.
    double angle;

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

		gamePadDriver  = new Joystick(1);
		gamePadShooter = new Joystick(2);

		limitSwitchShooter     = new DigitalInput(6);
		limitSwitchPickupLower = new DigitalInput(7);
		limitSwitchPickupUpper = new DigitalInput(8);

		ballPickupTimer = new Timer();

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
	}

	void TeleopInit(void) {
		// int m_telePeriodicLoops = 0; // Reset the loop counter for teleop mode
		// int m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
		for (int thisReading = 0; thisReading < numReadings; thisReading++) readings[thisReading] = 0; 
		compressor->Start();
		compressor_enabled = true;
		ballPickupTimer->Start();
		ballPickupTimer->Reset();
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {
	}

	void AutonomousPeriodic(void) {
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

	void TeleopPeriodic(void) {
		float leftStick  = -1*gamePadDriver->GetRawAxis(2);  // Drive system
		float rightStick = gamePadDriver->GetRawAxis(4);     // Drive system

		bool buttonX  = gamePadShooter->GetRawButton(1);     // Shooting
	    // bool buttonA  = gamePadShooter->GetRawButton(2);
		bool buttonB  = gamePadShooter->GetRawButton(3);     // Passing
		bool buttonY  = gamePadShooter->GetRawButton(4);     // Pickup abort button
		bool leftBumper  = gamePadDriver->GetRawButton(5);   // Shifting (Low)
		bool rightBumper = gamePadDriver->GetRawButton(6);   // Shifting (High)
		bool leftTrigger  = gamePadShooter->GetRawButton(7); // Gobbler position down
		bool rightTrigger = gamePadShooter->GetRawButton(8); // Gobbler position up
		// bool buttonBack = gamePadShooter->GetRawButton(9);
		bool buttonStart = gamePadShooter->GetRawButton(10); // Compressor

		// Compressor toggle
		if(buttonStart && buttonStartFlag) {
			if(compressor_enabled) {
				compressor->Stop();
				compressor_enabled = false;
			} else {
				compressor->Start();
				compressor_enabled = true;
			}
			buttonStartFlag = false;
		} else if (!buttonStart) {
			buttonStartFlag = true;
		}

		// Shifting
		if(rightBumper && !leftBumper) {
			ShiftHigh();
		} else if(leftBumper && !rightBumper) {
			ShiftLow();
		}

		// Gobbler positioning
		if((rightTrigger) && (!leftTrigger))
			armSpeed = 1.0;

	    if((!rightTrigger) && (leftTrigger))
			armSpeed = -1.0;

        if((!rightTrigger) && (!leftTrigger))
			armSpeed = 0.0;

		// Trigger for starting pickup sequence
		if(rightTrigger && armPickupFlag && (!fireFlag)) {
			armPickupFlag = false;
			timerFlag = false;
			pickupFlag = true;
			rollerSpeed = 0.4;
			pickupStage = 0;
			printf("Pickup triggered!\n");
		} else if (!rightTrigger) {
			armPickupFlag = true;
		}

		// Pickup abort
		if(buttonY && pickupFlag && (!abortFlag)) {
			pickupStage = 5; // Arbitrary value to go to abort sequence
			abortFlag = true;
			rollerSpeed = -0.4;
			printf("Pickup aborted!\n");
		} else if (!leftTrigger) {
			abortFlag = false;
		}

		// Firing
		if((!pickupFlag) && (buttonX) && (!fireFlag)) {
			fireFlag = true;
		}

		if(fireFlag && (!armFireFlag)) {
			// limit switch: true = not pressed, false = pressed
			if (limitSwitchPickupLower->Get()) {
				armSpeed = 1.0;
				rollerSpeed = 0.25;
			} else {
				armSpeed = 0.0;
				rollerSpeed = 0.0;
				armFireFlag = true;
			}
		}

		if(fireFlag && armFireFlag) {
			// limit switch: true = not pressed, false = pressed
			if (limitSwitchShooter->Get()) {
				shooterSpeed = 0.0;
				armSpeed = -1.0;

				if (limitSwitchPickupUpper->Get()) {
					armSpeed = 0.0;
					rollerSpeed = 0.0;
					fireFlag = false;
					armFireFlag = false;
				}
			} else {
				shooterSpeed = 1.0;
			}
		}

		// Pickup sequence
		if(pickupFlag) {
			if(pickupStage == 0) {
				printf("pickup stage = 0\n");
				// limit switch: true = not pressed, false = pressed
				if(limitSwitchPickupLower->Get()) {
					armSpeed = 0.0;
				} else {
					armSpeed = 1.0;
				}
			} else if (pickupStage == 5) { // Abortion sequence, started by buttonY
				// limit switch: true = not pressed, false = pressed
				if (limitSwitchPickupUpper->Get()) {
					armSpeed = 0.5;
					myTime = ballPickupTimer->Get();
				} else {
					armSpeed = 0.0;
					pickupFlag = false;
				}
			}
		}

		// Passing (reversing gobbler)
		if((!pickupFlag) && (buttonB)) {
			rollerSpeed = -1.0;
		} else if((!pickupFlag) && (!buttonB)) {
			rollerSpeed = 0.0;
		}

		// Print values (rate limited to 1/20)
//		if((printCounter % 20) == 0) {
//			printf("%d potentiometer: %d\n", printCounter, potentiometerValue);
//		}
//		printCounter++;

		// Motor speed declarations done at the end to ensure watchdog is continually updated.
		motorControlLeft(leftStick);
		motorControlRight(rightStick);
//      printf("Left: %f, Right: %f\n", leftStick, rightStick);
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

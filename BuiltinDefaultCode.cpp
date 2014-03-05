#include "WPILib.h"
#include <math.h>
#include <sstream>

#define TIME_WAIT 0.15      // Time delay before ball pickup
const int numReadings = 20; // Number of distance sensor readings to avg
const int threshold = 300;  // Distance sensor threshold
int readings[numReadings];  // Readings from the analog input
int sensorIndex = 0;        // Index of the current reading
int total = 0;              // Running total
int dist_average = 0;       // Distance sensor average
double myTime = 0;          // Timer for automatic ball pickup
double targetTime = 0;

// Flags
bool compressor_enabled = true;
bool buttonStartFlag = true;
bool armFlag = true;
bool timerFlag;
bool pickupFlag = false;
bool fireFlag = false;
bool positionFlag = false;

int pickupStage = 0;
int potentiometerValue;
int printCounter = 0;
float rollerSpeed = 0.0;
float positionSpeed = 0.0;
float shooterSpeed = 0.0;

class BuiltinDefaultCode : public IterativeRobot {

	// Motor controllers
	Talon *left_1;
	Talon *left_2;
	Talon *right_1;
	Talon *right_2;
	Victor *zoidbergRoller;
	Victor *zoidbergPosition;
	Victor *shooter;
	
	// Compressor
	Compressor *compressor;	
	
	// Solenoids
	Solenoid *shifterHi;
	Solenoid *shifterLo; 

	// Encoders
	Encoder *leftEncoder;
	Encoder *rightEncoder;

	// Sensors
	AnalogChannel *potentiometer;
	AnalogChannel *distanceSensor;

	// Axis Camera
	AxisCamera *camera;
	
	// Timer for automatic ball pickup
	Timer *timer;

	// Limit switches
	DigitalInput *limitSwitchShooter;

	// Joystick
	Joystick *gamePadDriver;  // Silver
	Joystick *gamePadShooter; // Red

	// Declare (x,y) coordinates of the robot on the field.
	double m_x;
	double m_y;

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

		zoidbergRoller = new Victor(5);
		shooter = new Victor(7);
		zoidbergPosition = new Victor(8);

		compressor = new Compressor(1,1);

		shifterHi = new Solenoid(1);
		shifterLo = new Solenoid(2);
				
		leftEncoder  = new Encoder(4, 5); // Second int is a placeholder to fix an error with the code (Encoder takes 2 ints)
		rightEncoder = new Encoder(1, 2); // Same here
		
		distanceSensor = new AnalogChannel(1);
		potentiometer  = new AnalogChannel(2);

		gamePadDriver  = new Joystick(1);
		gamePadShooter = new Joystick(2);

		limitSwitchShooter = new DigitalInput(6);
		
		timer = new Timer();

		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		// UINT32 m_priorPacketNumber;	// keep track of the most recent packet number from the DS
		// UINT8 m_dsPacketsReceivedInCurrentSecond; // keep track of the ds packets received in the current second

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		// m_autoPeriodicLoops = 0;
		// m_disabledPeriodicLoops = 0;
		// m_telePeriodicLoops = 0;

		m_x = 0;
		m_y = 0;

        // Set feet per pulse for encoder.
        leftEncoder->SetDistancePerPulse(.00460243323); //Not the actual value. Find the gear ratio.
        rightEncoder->SetDistancePerPulse(.00460243323);

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
		timer->Start();
		timer->Reset();
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {
	}

	void AutonomousPeriodic(void) {
		//initialShot(1); // 1 is a temporary value.
		//centerRobot();
		//seekAndDestroy();
		//shoot();
		//centerRobot();
		//0.,,,,,,,=0seekAndDestroy();
		//shoot();
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
		// shifter->Get() return value: false is low gear, true is high gear
		if(!(shifterHi->Get())) {
			shifterHi->Set(true);
			shifterLo->Set(false);
		}
	}

	void ShiftLow(void) {
		// shifter->Get() return value: false is low gear, true is high gear
		if(shifterHi->Get()) {
			shifterHi->Set(false);
			shifterLo->Set(true);
		}
	}

	void TeleopPeriodic(void) {
		total -= readings[sensorIndex];         
		//readings[sensorIndex] = distanceSensor->GetValue();
		readings[sensorIndex] = 1;
		total += readings[sensorIndex];       
		sensorIndex++;

		if (sensorIndex >= numReadings) {
			sensorIndex = 0;                           
		}
		dist_average = total / numReadings;
		
		float leftStick  = -1*gamePadDriver->GetRawAxis(2);
		float rightStick = gamePadDriver->GetRawAxis(4);
		bool buttonX  = gamePadShooter->GetRawButton(1);
	    bool buttonA  = gamePadShooter->GetRawButton(2);
		bool buttonB  = gamePadShooter->GetRawButton(3);
		bool buttonY  = gamePadShooter->GetRawButton(4);
		bool leftBumper  = gamePadDriver->GetRawButton(5);
		bool rightBumper = gamePadDriver->GetRawButton(6);
		bool leftTrigger  = gamePadShooter->GetRawButton(7);
		bool rightTrigger = gamePadShooter->GetRawButton(8);
		// bool buttonBack = gamePadShooter->GetRawButton(9);
		bool buttonStart = gamePadShooter->GetRawButton(10);
		

		// Compressor toggle
		if(buttonStart && buttonStartFlag) {
			if(compressor_enabled) {
				compressor->Stop();
				compressor_enabled = false;
			} 
			else {
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
		
		if((rightTrigger) && (!leftTrigger))
			positionSpeed = 1.0;
	
	    if((!rightTrigger) && (leftTrigger))
			positionSpeed = -1.0;

        if((!rightTrigger) && (!leftTrigger))
			positionSpeed = 0.0;
		
		// Passing (reversing roller)
		if((!pickupFlag) && (buttonB)) {
			rollerSpeed = -1.0;
		} else if((!pickupFlag) && (!buttonB)) {
			rollerSpeed = 0.0;
		}
		
		// Shooting
		if((!pickupFlag) && (buttonX)) {
			shooterSpeed = 1.0;
			// limit switch: true (1) = not pressed, false (0) = pressed
			// bool limitShooterValue = limitSwitchShooter->Get();
		} else if((!pickupFlag) && (!buttonX)) {
			shooterSpeed = 0.0;
		}
		
		if((!pickupFlag) && (buttonA)) {
			rollerSpeed = 1.0;
			// limit switch: true (1) = not pressed, false (0) = pressed
			// bool limitShooterValue = limitSwitchShooter->Get();
		} else if((!pickupFlag) && (!buttonA)) {
			shooterSpeed = 0.0;
		}
		

		// Trigger for starting pickup sequence
	/*	if(rightTrigger && armFlag) {
			armFlag = false;
			timerFlag = false;
			pickupFlag = true;
			rollerSpeed = 0.4;
			pickupStage = 0;
			printf("Pickup triggered\n");
		} else if (!rightTrigger) {
			armFlag = true;
		}*/
		
		potentiometerValue = potentiometer->GetValue();
		
		
		if((!pickupFlag) && (buttonY) && (!fireFlag)) {
			fireFlag = true;
		}
		
		if(fireFlag && (!positionFlag)){
			if (potentiometerValue < 38){
				positionSpeed = 1.0;
				rollerSpeed = 0.25;
			} else {
				positionSpeed = 0.0;
				rollerSpeed = 0.0;
				fireFlag = false;
				positionFlag = true;
			}
		}
		
		if(fireFlag && positionFlag){
			positionSpeed = -1.0;
			rollerSpeed = -0.25;
			if(potentiometerValue < 18){
				positionSpeed = 0.0;
				rollerSpeed = 0.0;
				fireFlag = false;
				positionFlag = false;
			}
		}
		
		
		
		// Pickup sequence
		/*if(pickupFlag){
			if(pickupStage == 0) {
				printf("pickup stage = 0\n");
				if(potentiometerValue < 38) {
					positionSpeed = 1.0; //notice
				} else {
					positionSpeed = 0.0;
					pickupStage++;
				}
			} else if(pickupStage == 1) {
				printf("pickup stage = 1\n");
				myTime = timer->Get();
				// When object detected inside threshold, set target time for automatic arm withdrawal
				if((average >= threshold) && (!timerFlag)){
					targetTime = myTime + TIME_WAIT;
					timerFlag = true;
				}

				// If ball is not continuously detected for the TIME_WAIT limit, reset timer
				if(timerFlag && (average < threshold)){
					timerFlag = false;
				}

				// If sensor successfully exceeds value for TIME_WAIT OR manual override, start moving arm
				// up and advance pickup stage.
				if((timerFlag && (myTime >= targetTime)) || leftTrigger){
					positionSpeed = -1.0;
					rollerSpeed = 0.5;
					pickupStage++;
				}
			} else if(pickupStage == 2) {
				printf("pickup stage = 2\n");
				// If potentiometer > setpoint, stop motor, stop roller, pickupFlag = false
				if(potentiometerValue < 21){
					rollerSpeed = 0.60;
				}
				
				if(potentiometerValue < 18){
					positionSpeed = 0.0;
					rollerSpeed = 0.0;
					pickupFlag = false;
				}
			}
		}*/
		
		
		
		// Print values (rate limited to 1/20)
		if((printCounter % 20) == 0) {
			printf("%d potentiometer: %d\n", printCounter, potentiometerValue);
			printf("%d distance sensor: %d\n", printCounter, dist_average);
		}
		printCounter++;

		// Third position for gobbler between 0 degrees and pickup position when firing
		// Gobbler runs showly when ball is on the catapult


		// Motor speed declarations done at the end to ensure watchdog is continually updated.
		motorControlLeft(leftStick);
		motorControlRight(rightStick);
		//printf("Left: %f, Right: %f\n", leftStick, rightStick);
	    zoidbergRoller->SetSpeed(rollerSpeed);
		zoidbergPosition->SetSpeed(positionSpeed);
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

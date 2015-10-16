package org.usfirst.frc.team449.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	//======================Universal constants====================
	
	public static final int DRIVE_ENCODER_CPR;
	public static final int ELEVATOR_ENCODER_CPR;
	
	//======================Elevator Constants=====================
	/**
	 * Channel for the elevator's top limit switch.
	 */
	public static final int ELEVATOR_TOP_LIMIT;

	/**
	 * Channel for the elevator's bottom limit switch.
	 */
	public static final int ELEVATOR_BOTTOM_LIMIT;
	
	/**
	 * Channel for the elevator's left limit switch.
	 */
	public static final int ELEVATOR_LEFT_LIMIT;
	
	/**
	 * Channel for the elevator's right limit switch.
	 */
	public static final int ELEVATOR_RIGHT_LIMIT;
	
	/***
	 * analog channel for elevator ultrasonic sensor
	 */
	public static final int ELEVATOR_ULTRASONIC;
	
	/**
	 * Channels for the elevator's arm solenoid forward thing.
	 */
	public static final int ELEVATOR_ARM_SOLENOID_FWD;
	
	/**
	 * Channels for the elevator's arm solenoid reverse thing.
	 */
	public static final int ELEVATOR_ARM_SOLENOID_REV;
	
	/**
	 * Channels for the elevator's brake solenoid forward thing.
	 */
	public static final int ELEVATOR_BRAKE_SOLENOID_FWD;
	
	/**
	 * Channels for the elevator's brake solenoid reverse thing.
	 */
	public static final int ELEVATOR_BRAKE_SOLENOID_REV;

	/**
	 * Channel A for the elevator's encoder.
	 */
	public static final int ELEVATOR_ENCODER_CHANNEL_A;
	
	/**
	 * Channel B for the elevator's encoder.
	 */
	public static final int ELEVATOR_ENCODER_CHANNEL_B;
	
	public static final double ELEVATOR_P;
	public static final double ELEVATOR_I;
	public static final double ELEVATOR_D;
	
	/**
	 * Port for the elevator's left motor
	 */
	public static final int ELEVATOR_LEFT_MOTOR;
	
	/**
	 * Port for the elevator's right motor
	 */
	public static final int ELEVATOR_RIGHT_MOTOR;
	
	public static final double ELEVATOR_PID_TOLERANCE_RANGE;
	
	/**
	 * initial encoder top count
	 */
	public static final double ELEVATOR_INITIAL_TOP_COUNT;
	
	public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE;
	
	//============================DriveSwerve Constants=======================
	/**
	 * motor controller channel for the left 1 drive motor
	 */
	public static final int DRIVE_L1;

	/**
	 * motor controller channel for the left 1 drive motor
	 */
	public static final int DRIVE_L2;

	/**
	 * motor controller channel for the right 1 drive motor
	 */
	public static final int DRIVE_R1;

	/**
	 * motor controller channel for the right 2 drive motor
	 */
	public static final int DRIVE_R2;
	
	/**
	 * Channel A for right encoder
	 */
	public static final int DRIVE_ENCODER_RA;
	
	/**
	 * channel B for right encoder
	 */
	public static final int DRIVE_ENCODER_RB;
	
	/**
	 * channel A for left encoder
	 */
	public static final int DRIVE_ENCODER_LA;
	
	/**
	 * channel B for left encoder
	 */
	public static final int DRIVE_ENCODER_LB;

	/**
	 * DriveSwerve PID P term
	 */
	public static final double DRIVE_P;
	
	/**
	 * DriveSwerve PID I term
	 */
	public static final double DRIVE_I;
	
	/**
	 * DriveSwerve PID D term
	 */
	public static final double DRIVE_D;
	
	/**
	 * DriveSwerve PID F term for speed control
	 */
	public static final double DRIVE_F;

	/**
	 * Hard limit for DriveSwerve speed under PID control in Rotations per second
	 */
	public static final int DRIVE_MAX_RATE;
	
	/**
	 * whether default drive is manual or not (if not then PID)
	 */
	public static final boolean DRIVE_DEFAULT_MANUAL;
	
	//===========================Intake constants========================
	/**
	 * Channel for the intake's left limit switch.
	 */
	public static final int INTAKE_LEFT_LIMIT;
	
	/**
	 * Channel for the intake's right limit switch.
	 */
	public static final int INTAKE_RIGHT_LIMIT;
	
	/**
	 * Channel for the intake's left motor.
	 */
	public static final int INTAKE_LEFT_MOTOR;
	
	/**
	 * Channel for the intake's right motor.
	 */
	public static final int INTAKE_RIGHT_MOTOR;
	
	public static final int INTAKE_LED_PORT;
	
	/**
	 * left intake forward solenoid channel
	 */
	public static final int INTAKE_LSOLENOID_FORWARD;
	
	/**
	 * left intake reverse solenoid channel
	 */
	public static final int INTAKE_LSOLENOID_REVERSE;
	
	/**
	 * right intake forward solenoid channel
	 */
	public static final int INTAKE_RSOLENOID_FORWARD;
	
	/**
	 * right intake reverse solenoid channel
	 */
	public static final int INTAKE_RSOLENOID_REVERSE;

	public static final int INTAKE_ULTRASONIC;
	public static final int INTAKE_JOYSTICK;
	//===========================Controller Ports/Scheme======================
	/**
	 * Joystick usb port for Joystick 0
	 */
	public static final int JOYSTICK_0;
	
	/**
	 * Joystick usb port for Joystick 1
	 */
	public static final int JOYSTICK_1;
	
	/**
	 * Joystick usb port for Joystick 2
	 */
	public static final int JOYSTICK_2;
	
	/**
	 * Joystick usb port for Joystick 3
	 */
	public static final int JOYSTICK_3;
	
	/**
	 * the joystick with the button to toggle manual mode for the drive system
	 */
	public static final int DRIVE_MANUAL_TOGGLE_JOYSTICK;
	

	/**
	 * the button number on DRIVE_MANUAL_TOGGLE_JOYSTICK that toggles manual mode for the drive system
	 */
	public static final int DRIVE_MANUAL_TOGGLE_BUTTON;
	
	/**
	 * Button for moving elevator up.
	 */
	public static final int ELEVATOR_UP_BUTTON;
	
	/**
	 * Button for moving elevator down.
	 */
	public static final int ELEVATOR_DOWN_BUTTON;

	/**
	 * Button to open the elevator arms
	 */
	public static final int ELEVATOR_ARMS_OPEN_BUTTON;

	/**
	 * Button to close the elevator arms
	 */
	public static final int ELEVATOR_ARMS_CLOSE_BUTTON;

	/**
	 * 
	 */
	public static final int ELEVATOR_MOVE_JOYSTICK;
	
	public static final int DRIVE_LEFT_JOYSTICK;
	
	public static final int DRIVE_RIGHT_JOYSTICK;

	/**
	 * Joystick for swerve control
	 */
	public static final int DRIVE_SWERVE_JOYSTICK;
	
	public static final double DRIVE_CONTROL_SENSITIVITY;
	
	public static final int INTAKE_ARMS_OPEN;
	public static final int INTAKE_ARMS_CLOSE;
	
	/**
	 * 
	 * @param configFile
	 */
	static {
		System.out.println("RobotMap");
		DRIVE_ENCODER_CPR = 256;
		ELEVATOR_ENCODER_CPR = 512;
		
		//==================================================Elevator Constants ==================================================
		
		ELEVATOR_TOP_LIMIT 	= 9;
		ELEVATOR_BOTTOM_LIMIT 	= 8;
		
		ELEVATOR_LEFT_LIMIT 	= 10;
		ELEVATOR_RIGHT_LIMIT 	= 6;
		
		ELEVATOR_ULTRASONIC = 0;
		
		ELEVATOR_ARM_SOLENOID_FWD = 2;
		ELEVATOR_ARM_SOLENOID_REV = 3;
		
		ELEVATOR_BRAKE_SOLENOID_FWD = 1;
		ELEVATOR_BRAKE_SOLENOID_REV = 0;
		
		ELEVATOR_ENCODER_CHANNEL_A = 4;
		ELEVATOR_ENCODER_CHANNEL_B = 5;
		
		ELEVATOR_P = 0.005;
		ELEVATOR_I = 0;
		ELEVATOR_D = 0;
		
		ELEVATOR_LEFT_MOTOR = 2;
		ELEVATOR_RIGHT_MOTOR = 3;
		
		ELEVATOR_PID_TOLERANCE_RANGE = 0.15;
		
		ELEVATOR_INITIAL_TOP_COUNT = 2000;
		ELEVATOR_SPROCKET_CIRCUMFERENCE = 3*Math.PI;
		
		//==================================================DriveSwerve Constants ==================================================
		
		DRIVE_L1 = 6;
		DRIVE_L2 = 7;
		
		DRIVE_R1 = 4;
		DRIVE_R2 = 5;
		
		DRIVE_ENCODER_LA = 2;
		DRIVE_ENCODER_LB = 3;
		
		DRIVE_ENCODER_RA = 0;
		DRIVE_ENCODER_RB = 1;
		
		DRIVE_P = 0.005;
		DRIVE_I = 0;
		DRIVE_D = 0;
		DRIVE_F = 0;
		
		DRIVE_MAX_RATE = 4;
		DRIVE_DEFAULT_MANUAL = true;
		
		//==================================================Intake Constants ==================================================
		
		INTAKE_LEFT_LIMIT = 7;
		INTAKE_RIGHT_LIMIT = 7;
		
		INTAKE_LEFT_MOTOR = 8;
		INTAKE_RIGHT_MOTOR = 8;
		INTAKE_LED_PORT =9;
		
		INTAKE_LSOLENOID_FORWARD = 4;
		INTAKE_LSOLENOID_REVERSE = 5;
		
		INTAKE_RSOLENOID_FORWARD = 7;
		INTAKE_RSOLENOID_REVERSE = 6;
		
		INTAKE_ULTRASONIC = 0;
		//==================================================Control Constants ==================================================
		
		JOYSTICK_0 = 0;
		JOYSTICK_1 = 1;
		JOYSTICK_2 = 2;
		JOYSTICK_3 = 3;

		DRIVE_LEFT_JOYSTICK 	= JOYSTICK_0;
		DRIVE_RIGHT_JOYSTICK 	= JOYSTICK_1;

		DRIVE_SWERVE_JOYSTICK = JOYSTICK_0;

		DRIVE_CONTROL_SENSITIVITY = 0.5;

		DRIVE_MANUAL_TOGGLE_JOYSTICK 	= DRIVE_LEFT_JOYSTICK;
		DRIVE_MANUAL_TOGGLE_BUTTON		= 1;
		
		ELEVATOR_UP_BUTTON = 2;
		ELEVATOR_DOWN_BUTTON = 3;
		
		ELEVATOR_ARMS_OPEN_BUTTON = 1;
		ELEVATOR_ARMS_CLOSE_BUTTON = 3;
		
		ELEVATOR_MOVE_JOYSTICK = JOYSTICK_2;

		INTAKE_JOYSTICK = JOYSTICK_3;
		INTAKE_ARMS_CLOSE = 1;
		INTAKE_ARMS_OPEN = 2;
	}//end RobotMap()
}//end class

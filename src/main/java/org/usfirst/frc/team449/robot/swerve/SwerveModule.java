package org.usfirst.frc.team449.robot.swerve;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team449.robot.OIMap;
import org.usfirst.frc.team449.robot.swerve.components.PIDMotor;

/**
 * A class to control each module of the swerve drive
 * The physical swerve module has 2 motor and encoder pairs
 * One motor-encoder pair controls the rotation of the wheel around the vertical axis (ie where the wheel is facing)
 * The other motor-encoder pair controls the speed of the wheels rotation around a horizontal axis, directly affecting the speed of the wheel
 *
 * The former will most easily and intuitively be controlled via its PID controller, as we are approaching a setpoint that the encoder can easily measure
 * The latter, as the team had discovered in 2015 (Recycle Rush) is very hard to control via PID, so it will likely be controlled via direct voltage
 *
 * That said, PID control for either should be programmatically toggleable, and this class will have the architecture to support any configuration of PID and non-PID
 */
public class SwerveModule {

	/**
	 * the motor to control the velocity of this module's wheel
	 */
	private final SpeedController velocityMotor;
	/**
	 * the motor to control the rotation of this module's wheel
	 */
	private final SpeedController rotationMotor;

	/**
	 * the encoder for the motor controlling the velocity of this module's wheel
	 */
	private final Encoder velocityEncoder;
	/**
	 * the encoder for the motor controlling the rotation of this module's wheel
	 */
	private final Encoder rotationEncoder;

	/**
	 * PID controller for this module's wheel's velocity
	 */
	private final PIDMotor velocityController;
	/**
	 * PID controller for this module's wheel's rotatiion
	 */
	private final PIDController rotationController;

	/**
	 * the previous value the velocity was set to
	 */
	private double lastSetVelocity;
	/**
	 * the previous value the rotation was set to
	 */
	private double lastSetRotation;
	/**
	 * whether the velocity is controlled manually (false for PID)
	 */
	private static boolean manualVelocity;
	/**
	 * whether rotation is controlled manually (false for PID)
	 */
	private static boolean manualRotation;
	
	private static final int ROTATION_MOTOR_VELOCITY = 10;

	/**
	 * Constructs a module with the motors and encoders of this module, and internally creates PID controllers for rotation and velocity
	 * @param velocityMotor the motor to control the velocity of this module's wheel
	 * @param velocityEncoder the encoder for the motor controlling the velocity of this module's wheel
	 * @param rotationMotor the motor to control the rotation of this module's wheel
	 * @param rotationEncoder the encoder for the motor controlling the rotation of this module's wheel
	 */
	public SwerveModule(SpeedController velocityMotor, Encoder velocityEncoder, boolean isManualVelocity, SpeedController rotationMotor, Encoder rotationEncoder, boolean isManualRotation)
	{
		this.velocityMotor = velocityMotor;
		this.velocityEncoder = velocityEncoder;
		this.rotationMotor = rotationMotor;
		this.rotationEncoder = rotationEncoder;

		this.velocityController = new PIDMotor(SwerveMap.P, SwerveMap.I, SwerveMap.D, 0, SwerveMap.TOLERANCE, velocityMotor ,velocityEncoder, PIDMotor.SPEED_BASE);
		this.rotationController = new PIDController(SwerveMap.P, SwerveMap.I, SwerveMap.D, SwerveMap.F, rotationEncoder, rotationMotor);

		this.lastSetVelocity = 0;
		this.lastSetRotation = 0;
	}

	/**
	 * sets the rotation of the wheel
	 * @param rotationDegrees rotation in degrees where 0 points the wheel to the robot's front
	 */
	public void rotate(double rotationDegrees) {
		
		lastSetRotation = rotationDegrees;
		
		if(isManualRotation()){
			while(rotationEncoder.get() > lastSetRotation)
				rotationMotor.set(-ROTATION_MOTOR_VELOCITY);
			while(rotationEncoder.get() < lastSetRotation)
				rotationMotor.set(ROTATION_MOTOR_VELOCITY);
		}
		else{
			rotationController.setSetpoint(lastSetRotation);
		}	
	}

	/**
	 * sets the velocity of the wheel
	 * @param velocity velocity of the motor
	 */
	public void set(double velocity) {
		lastSetVelocity = velocity;
		if(!isManualVelocity())
			velocityController.setSetpoint(lastSetVelocity);
		velocityController.setMotorVoltage(lastSetVelocity);
	}

	/**
	 * sets the heading of the module, robot centric
	 * @param speed velocity of the wheel
	 * @param rotationDegrees rotation in degrees where 0 points the wheel to the robot's front
	 */
	public void goTo(double speed, double rotationDegrees) {
		set(speed);
		rotate(rotationDegrees);
	}

	/**
	 * whether the velocity is controlled manually (PID otherwise)
	 * @return true if velocity of all modules is controlled manually, and false otherwise
	 */
	public static boolean isManualVelocity() {
		return manualVelocity;
	}

	/**
	 * whether the rotation is controlled manually (PID otherwise)
	 * @return true if rotation of all modules is controlled manually, and false otherwise
	 */
	public static boolean isManualRotation() {
		return manualRotation;
	}

	/**
	 * whether the velocity should be controlled manually (PID otherwise)
	 * enables or disables the PID controller accordingly
	 * @param isOn true if velocity of all modules should be controlled manually, and false otherwise
	 */
	public void setManualVelocity(boolean isOn) {
		manualVelocity = isOn;
		if(manualVelocity) 
			velocityController.disable();
		else
			velocityController.enable();
	}

	/**
	 * whether the rotation should be controlled manually (PID otherwise)
	 * enables or disables the PID controller accordingly
	 * @param isOn true if rotation of all modules should be controlled manually, and false otherwise
	 */
	public void setManualRotation(boolean isOn) {
		manualRotation = isOn;
		if(manualRotation) 
			rotationController.disable();
		else
			rotationController.enable();
	}
}//end class
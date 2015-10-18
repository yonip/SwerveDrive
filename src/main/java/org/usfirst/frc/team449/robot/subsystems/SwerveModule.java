package org.usfirst.frc.team449.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team449.robot.RobotMap;

import java.util.ArrayList;

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
 * TODO: use two boolean flags to toggle PID for rotation and velocity
 */
public class SwerveModule extends Subsystem {

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
	private final PIDController velocityController;
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
	 * Constructs a module with the motors and encoders of this module, and internally creates PID controllers for rotation and velocity
	 * @param velocityMotor the motor to control the velocity of this module's wheel
	 * @param velocityEncoder the encoder for the motor controlling the velocity of this module's wheel
	 * @param rotationMotor the motor to control the rotation of this module's wheel
	 * @param rotationEncoder the encoder for the motor controlling the rotation of this module's wheel
	 */
	public SwerveModule(SpeedController velocityMotor, Encoder velocityEncoder, SpeedController rotationMotor, Encoder rotationEncoder)
	{
		this.velocityMotor = velocityMotor;
		this.velocityEncoder = velocityEncoder;
		this.rotationMotor = rotationMotor;
		this.rotationEncoder = rotationEncoder;

		this.velocityController = new PIDController(RobotMap.DRIVE_P, RobotMap.DRIVE_I, RobotMap.DRIVE_D, RobotMap.DRIVE_F, velocityEncoder, velocityMotor);
		this.rotationController = new PIDController(RobotMap.DRIVE_P, RobotMap.DRIVE_I, RobotMap.DRIVE_D, RobotMap.DRIVE_F, rotationEncoder, rotationMotor);

		this.lastSetVelocity = 0;
		this.lastSetRotation = 0;
	}

	/**
	 * sets the rotation of the wheel
	 * @param rotationDegrees rotation in degrees where 0 points the wheel to the robot's front
	 */
	public void rotate(double rotationDegrees) {
		// TODO: implement
	}

	/**
	 * sets the velocity of the wheel
	 * @param speed velocity of the motor
	 */
	public void set(double speed) {
		// TODO: implement
	}

	/**
	 * sets the heading of the module
	 * @param speed velocity of the wheel
	 * @param rotationDegrees rotation in degrees where 0 points the wheel to the robot's front
	 */
	public void goTo(double speed, double rotationDegrees) {
		// TODO: implement, maybe rename
	}

	@Override
	protected void initDefaultCommand() {
		//no default command yo
	}
}//end class
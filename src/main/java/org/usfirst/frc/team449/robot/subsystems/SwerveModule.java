package org.usfirst.frc.team449.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team449.robot.RobotMap;

import java.util.ArrayList;

/**
 *
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

	private double lastSet = 0;

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

		this.lastSet = 0;
	}

	/**
	 * sets the rotation of the wheel
	 * @param rotationDegrees rotation in degrees where 0 points the wheel to the robot's front
	 */
	public void rotate(double rotationDegrees) {

	}

	/**
	 * sets the velocity of the wheel
	 * @param speed velocity of the motor
	 */
	public void set(double speed) {

	}

	/**
	 * sets the heading of the module
	 * @param speed velocity of the wheel
	 * @param rotationDegrees rotation in degrees where 0 points the wheel to the robot's front
	 */
	public void goTo(double speed, double rotationDegrees) {

	}

	@Override
	protected void initDefaultCommand() {
		//no default command yo
	}
}//end class
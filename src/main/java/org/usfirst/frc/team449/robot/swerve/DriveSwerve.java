package org.usfirst.frc.team449.robot.swerve;

import org.usfirst.frc.team449.robot.OIMap;
import org.usfirst.frc.team449.robot.swerve.commands.DriveSwerveRobot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team449.robot.swerve.components.MotorCluster;

import java.awt.*;

/**
 * The DriveSwerve subsystem.
 */
public class DriveSwerve extends Subsystem {
	/**
	 * SwerveModule that controls the speed and rotation of the front-left wheel
	 */
	private final SwerveModule frontLeftMotors;
	/**
	 * SwerveModule that controls the speed and rotation of the front-right wheel
	 */
	private final SwerveModule frontRightMotors;
	/**
	 * SwerveModule that controls the speed and rotation of the front-left wheel
	 */
	private final SwerveModule backLeftMotors;
	/**
	 * SwerveModule that controls the speed and rotation of the front-right wheel
	 */
	private final SwerveModule backRightMotors;
	/**
	 * the point around which the robot should rotate, relative to the robot
	 * The point is defined such that (0,0) is the center of the robot, (1,0) is one meter to the right and (0,1) is one meter forward
	 */
	private Point rotationPoint;
	
	/**
	 * Initialize the DriveSwerve subsystem
	 */
	public DriveSwerve(){
		System.out.println("DriveSwerve init started");

		this.frontLeftMotors = generateModule(SwerveMap.Motors.Velocity.FRONT_LEFT, SwerveMap.Encoders.Velocity.FRONT_LEFT_A,
				SwerveMap.Encoders.Velocity.FRONT_LEFT_B, SwerveMap.Motors.Rotater.FRONT_LEFT, SwerveMap.Encoders.Rotater.FRONT_LEFT_A,
				SwerveMap.Encoders.Rotater.FRONT_LEFT_B);
		this.frontRightMotors = generateModule(SwerveMap.Motors.Velocity.FRONT_RIGHT, SwerveMap.Encoders.Velocity.FRONT_RIGHT_A,
				SwerveMap.Encoders.Velocity.FRONT_RIGHT_B, SwerveMap.Motors.Rotater.FRONT_RIGHT, SwerveMap.Encoders.Rotater.FRONT_RIGHT_A,
				SwerveMap.Encoders.Rotater.FRONT_RIGHT_B);
		this.backLeftMotors = generateModule(SwerveMap.Motors.Velocity.BACK_LEFT, SwerveMap.Encoders.Velocity.BACK_LEFT_A,
				SwerveMap.Encoders.Velocity.BACK_LEFT_B, SwerveMap.Motors.Rotater.BACK_LEFT, SwerveMap.Encoders.Rotater.BACK_LEFT_A,
				SwerveMap.Encoders.Rotater.BACK_LEFT_B);
		this.backRightMotors = generateModule(SwerveMap.Motors.Velocity.BACK_RIGHT, SwerveMap.Encoders.Velocity.BACK_RIGHT_A,
				SwerveMap.Encoders.Velocity.BACK_RIGHT_B, SwerveMap.Motors.Rotater.BACK_RIGHT, SwerveMap.Encoders.Rotater.BACK_RIGHT_A,
				SwerveMap.Encoders.Rotater.BACK_RIGHT_B);
		System.out.println("DriveSwerve init finished");
	}//end drive

	/**
	 * sets the heading and rotation of the robot, field centric
	 * rotation will be around an arbitrary point stored in this class (rotationPoint)
	 * @param xVelocity the x component of the vector for the robot to move in
	 * @param yVelocity the y component of the vector for the robot to move in
	 * @param rotationRadians how fast the robot should rotate in radians/second
	 * @see
	 */
	public void goTo(double xVelocity, double yVelocity, double rotationRadians){
		// TODO implement, glhf
	}


	/**
	 * the point around which the robot should rotate, relative to the robot
	 * The point is defined such that (0,0) is the center of the robot, (1,0) is one meter to the right and (0,1) is one meter forward
	 * @param p the point around which to rotate, relative to the robot
	 */
	public void setRotationPoint(Point p) {
		this.rotationPoint = p;
	}

	/**
	 * the point around which the robot will rotate, relative to the robot
	 * The point is defined such that (0,0) is the center of the robot, (1,0) is one meter to the right and (0,1) is one meter forward
	 * @return the point around which the robot will rotate, relative to the robot
	 */
	public Point getRotationPoint() {
		return this.rotationPoint;
	}

    public void initDefaultCommand() {
    	setDefaultCommand(new DriveSwerveRobot());
    }

	/**
	 * toggles the control mode for velocity (manual or PID)
	 */
	public void toggleModeVelocity() {
		this.setManualVelocity(!this.isManualVelocity());
	}

	/**
	 * toggles the control mode for rotation (manual or PID)
	 */
	public void toggleModeRotation() {
		this.setManualRotation(!this.isManualRotation());
	}

	/**
	 * whether the velocity should be controlled manually (PID otherwise)
	 * @param isOn true if velocity of all modules should be controlled manually, and false otherwise
	 */
    public void setManualVelocity(boolean isOn)
    {
		SwerveModule.setManualVelocity(isOn);
    }

	/**
	 * whether the rotation should be controlled manually (PID otherwise)
	 * @param isOn true if rotation of all modules should be controlled manually, and false otherwise
	 */
	public void setManualRotation(boolean isOn) {
		SwerveModule.setManualRotation(isOn);
	}

	/**
	 * returns whether the velocity of the modules is controlled manually (otherwise, it's controlled via PID)
	 * @return true if the velocity of the modules is controlled manually, and false otherwise
	 */
	public boolean isManualVelocity()
	{
		return SwerveModule.isManualVelocity();
	}

	/**
	 * returns whether the rotation of the modules is controlled manually (otherwise, it's controlled via PID)
	 * @return true if the rotation of the modules is controlled manually, and false otherwise
	 */
	public boolean isManualRotation()
	{
		return SwerveModule.isManualRotation();
	}

	/**
	 * takes the ports for the motors and encoders and return an initialized SwerveModule with motors and encoders at the corresponding ports
	 * encoders have their DPP and PIDSourceParameter set
	 * @param velocityMotorPort port for the velocity motor
	 * @param velocityEncoderPortA port A of the encoder for velocity
	 * @param velocityEncoderPortB port b of the encoder for velocity
	 * @param rotationMotorPort port for the rotation motor
	 * @param rotationEncoderPortA port A of the encoder for rotation
	 * @param rotationEncoderPortB port B of the encoder for rotation
	 * @return a SwerveModule object with encoders and motors set using the given ports
	 */
	protected static SwerveModule generateModule(int velocityMotorPort, int velocityEncoderPortA, int velocityEncoderPortB, int rotationMotorPort, int rotationEncoderPortA, int rotationEncoderPortB) {
		Encoder velEnc = new Encoder(velocityEncoderPortA, velocityEncoderPortB);
		Encoder rotEnc = new Encoder(rotationEncoderPortA, rotationEncoderPortB);

		velEnc.setDistancePerPulse(SwerveMap.WHEEL_DIAMETER / SwerveMap.Encoders.Velocity.CPR);
		rotEnc.setDistancePerPulse(1.0 / SwerveMap.Encoders.Rotater.CPR); // because we just want it to go to radians, so distance should equal angle

		velEnc.setPIDSourceParameter(PIDSourceParameter.kRate);
		rotEnc.setPIDSourceParameter(PIDSourceParameter.kRate);

		return new SwerveModule(new VictorSP(velocityMotorPort), velEnc, SwerveMap.DEFAULT_VELOCITY_MANUAL, new VictorSP(rotationMotorPort), rotEnc, SwerveMap.DEFAULT_ROTATION_MANUAL);
	}
}//end class


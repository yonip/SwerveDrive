package org.usfirst.frc.team449.robot.swerve.commands;

import org.usfirst.frc.team449.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Autonomous drive command
 *
 */
public class AutoDriveSwerve extends Command {
	/**
	 * number between -1 and 1 denoting magnitude of angular velocity (negative is counter-clockwise)
	 */
	private double rotation;
	/**
	 * how far the robot should go (meters)
	 */
	private double distance;
	/**
	 * where the robt is headed, field centric (rad)
	 */
	private double heading;
	/**
	 * number between -1 and 1 denoting magnitude of velocity vector
	 */
	private double magnitude;
	/**
	 * hard time limit for operation (seconds)
	 */
	private double time;
	private Timer t;

	/**
	 * move the robot at rotation degrees relative to the driver (0 is forward) until the robot covers the given distance or time time has passed
	 * emulate the driver giving these inputs
	 * @param heading angle, in degrees, where the robot should head to, where 0 is forward relative to the driver (converted to radians for storage)
	 * @param magnitude number between -1 and 1 denoting magnitude of velocity vector
	 * @param rotation number between -1 and 1 denoting magnitude of angular velocity (negative is counter-clockwise)
	 * @param distance distance to travel (meters)
	 * @param time hard time limit
	 */
	public AutoDriveSwerve(double heading, double magnitude, double rotation, double distance, double time) {
		requires(Robot.drive);
		// conversion to radians
		this.heading = Math.PI*heading/180.0;
		this.magnitude = magnitude;
		this.rotation = rotation;
		this.distance = distance;
		this.time = time;
		t = new Timer();
		// might want to turn heading and magnitude to x magnitude and y magnitude. see execute() in DriveSwerveRobot
	}

	/**
	 * reset and start timer,
	 */
	protected void initialize() {
		// TODO implement
	}

	/**
	 * hurr durr I'm really a player, right? (ie pretty much use execute in DriveSwerveRobot)
	 */
	protected void execute() {
		// checklist:
		// 	x magnitude
		// 	y magnitude
		//  rotation magnitude
		// TODO implement
	}

	/**
	 * returns true once you hit your desired setpoint or time was exceeded
	 */
	protected boolean isFinished() {
		// TODO implement
	}

	/**
	 * probably stop the robot here
	 */
	protected void end() {
		// TODO implement
	}

	/**
	 * try setting velocity one last time
	 */
	protected void interrupted() {
		// TODO implement
	}
}

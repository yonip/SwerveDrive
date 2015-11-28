package main.java.org.usfirst.frc.team449.robot.swerve.commands;

import main.java.org.usfirst.frc.team449.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import main.java.org.usfirst.frc.team449.robot.swerve.SwerveMap;

/**
 * Autonomous drive command: drive a set distance (move the rotation point a set distance) and
 * rotate the robot around the rotation point by a set angle
 */
public class AutoDriveSwerve extends Command {
	
	/** number between -1 and 1 denoting magnitude of angular velocity (negative is counter-clockwise) */
	private double angularVel;
	/** how far the robot should go (meters) */
	private double distance;
	/** How much the robot should rotate around it's rotation point (radians) */
	private double rotation;
	/** where the robot is headed, field centric (radians) */
	private double heading;
	/** number between -1 and 1 denoting magnitude of velocity vector */
	private double magnitude;
	/** timer to keep track of how long the robot has been driving */
	private Timer t;
	
	/** how far the robot has traveled (meters) */
	private double distanceTraveled;
	/** the amount the robot has rotated (radians) */
	private double amountRotated;

	/**
	 * move the robot towards <code>heading</code> degrees relative to the driver (0 is forward) until the robot
	 * 		covers the given distance and rotates the given amount around the (moving) rotation point or 
	 * 		<code>time</code> seconds have passed
	 * @param heading angle, in degrees, where the robot should head to, where 0 is forward relative to 
	 * 		the driver (converted to radians for storage)
	 * @param magnitude number between -1 and 1 denoting magnitude of velocity vector
	 * @param angularVel number between -1 and 1 denoting magnitude of angular velocity (negative 
	 * 		is counter-clockwise)
	 * @param distance distance to travel (meters)
	 * @param rotation amount to rotate (degrees)
	 * @param time hard time limit (seconds)
	 */
	public AutoDriveSwerve(double heading, double magnitude, double angularVel, double distance, double rotation, double time) {
		super(time);
		requires(Robot.drive);
		this.heading = Math.toRadians(heading);
		this.magnitude = magnitude;
		this.angularVel = angularVel;
		this.distance = distance;
		this.rotation = Math.toRadians(rotation);
		t = new Timer();
	}

	/**
	 * start driving in given direction at given linear and angular velocity, and start recording how long
	 * the robot's been driving
	 */
	protected void initialize() {
		Robot.drive.goTo(magnitude * Math.cos(heading), magnitude * Math.sin(heading), angularVel);
		t.start();
	}

	/**
	 * update the distance traveled and the amount the robot has rotated since the robot started moving
	 * Also, set <code>magnitude</code> to 0 when the robot has moved the desired amount and
	 * <code>angularVel</code> to 0 when the robot has rotated the desired amount 
	 */
	protected void execute() {
		distanceTraveled = magnitude * t.get() * SwerveMap.DEFAULT_MAX_LINEAR_VELOCITY;
		amountRotated = angularVel * t.get() * SwerveMap.DEFAULT_MAX_ANGULAR_VELOCITY;
		
		if (distanceTraveled >= distance) {
			magnitude = 0;
		}
		if (amountRotated >= rotation) {
			angularVel = 0;
		}
	}

	/**
	 * @return true once both <code>magnitude</code> and <code>angularVel</code> are 0, since 
	 * {@link #execute() execute} will have set them to 0.
	 */
	protected boolean isFinished() {
		return magnitude == 0 && angularVel == 0;
	}

	/**
	 * Set all motors to 0
	 */
	protected void end() {
		Robot.drive.goTo(0, 0, 0);
	}

	/**
	 * Stop the robot!!! calls {@link #end() end}
	 */
	protected void interrupted() {
		end();
	}
}

package org.usfirst.frc.team449.robot.swerve.commands;

import edu.wpi.first.wpilibj.command.StartCommand;

public class AutoTranslateSwerve extends StartCommand {
	
	/**
	 * move the robot towards <code>heading</code> degrees relative to the driver (0 is forward) until the robot
	 * 		covers the given distance or <code>time</code> seconds have passed
	 * @param heading angle, in degrees, where the robot should head to, where 0 is forward relative to 
	 * 		the driver (converted to radians for storage)
	 * @param magnitude number between -1 and 1 denoting magnitude of velocity vector
	 * @param distance distance to travel (meters)
	 * @param time hard time limit (seconds)
	 */
	public AutoTranslateSwerve(double heading, double magnitude, double distance, double time) {
		super(new AutoDriveSwerve(heading, magnitude, 0, distance, 0, time));
	}

}

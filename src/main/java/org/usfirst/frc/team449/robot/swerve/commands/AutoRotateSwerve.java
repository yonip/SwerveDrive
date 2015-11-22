package org.usfirst.frc.team449.robot.swerve.commands;

import edu.wpi.first.wpilibj.command.StartCommand;

public class AutoRotateSwerve extends StartCommand {
	
	/**
	 * rotate the robot around the rotation point until it rotates the desired amount or
	 * 		<code>time</code> seconds have passed
	 * @param angularVel number between -1 and 1 denoting magnitude of angular velocity (negative 
	 * 		is counter-clockwise)
	 * @param rotation amount to rotate (degrees)
	 * @param time hard time limit (seconds)
	 */
	public AutoRotateSwerve(double angularVel, double rotation, double time) {
		super(new AutoDriveSwerve(0, 0, angularVel, 0, rotation, time));
	}

}

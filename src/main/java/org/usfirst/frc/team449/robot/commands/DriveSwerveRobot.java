package org.usfirst.frc.team449.robot.commands;

import org.usfirst.frc.team449.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveSwerveRobot extends Command {

    public DriveSwerveRobot() {
    	requires(Robot.drive);
    	System.out.println("DriveSwerve Robot bueno");
    }

    protected void initialize() {
    }

    protected void execute() {
    	double pitch = Robot.OI.getSwerveStickPitch();
    	double roll = Robot.OI.getSwerveStickRoll();
        double yaw = Robot.OI.getSwerveStickYaw();

    	// figure out swerve controls here
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
    	// figure out swerve controls here
    }
}
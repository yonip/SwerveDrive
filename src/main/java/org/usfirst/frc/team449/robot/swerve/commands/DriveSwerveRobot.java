package org.usfirst.frc.team449.robot.swerve.commands;

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
    	double x = Robot.OI.getSwerveStickX();
    	double y = Robot.OI.getSwerveStickY();
        double theta = Robot.OI.getSwerveStickTheta();

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
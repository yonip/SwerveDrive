package main.java.org.usfirst.frc.team449.robot.swerve.commands;

import main.java.org.usfirst.frc.team449.robot.Robot;
import main.java.org.usfirst.frc.team449.robot.swerve.SwerveMap;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Command to drive the robot given input from the driver
 */
public class DriveSwerveRobot extends Command {

    public DriveSwerveRobot() {
    	requires(Robot.drive);
    	System.out.println("DriveSwerve Robot bueno yaaay");
    }

    protected void initialize() {
    }

    protected void execute() {
    	//Set values to the values given by the joysticks, converted to velocities
        double x = Robot.OI.getSwerveStickX() * SwerveMap.DEFAULT_MAX_LINEAR_VELOCITY;
        double y = Robot.OI.getSwerveStickY() * SwerveMap.DEFAULT_MAX_LINEAR_VELOCITY;
        double theta = Robot.OI.getSwerveStickTheta(); 

        Robot.drive.goTo(x, y, theta);
    }

    protected boolean isFinished() {
    	return false;
    }

    protected void end() {
    	Robot.drive.goTo(0, 0, 0);
    }

    protected void interrupted() {
    	// TODO pretty much just execute one more time, probably
    }
}
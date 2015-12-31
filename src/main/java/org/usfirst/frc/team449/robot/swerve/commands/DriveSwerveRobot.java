package org.usfirst.frc.team449.robot.swerve.commands;

import org.usfirst.frc.team449.robot.Robot;
import org.usfirst.frc.team449.robot.swerve.SwerveMap;
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
        double x = Robot.oi.getSwerveStickX() * SwerveMap.DEFAULT_MAX_LINEAR_VELOCITY;
        double y = Robot.oi.getSwerveStickY() * SwerveMap.DEFAULT_MAX_LINEAR_VELOCITY;
        double omega = Robot.oi.getSwerveStickTheta();

        double magnitude = Math.sqrt(x*x+y*y);
        double theta = Math.atan2(y,x);

        Robot.drive.goTo(magnitude, theta, omega);
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        Robot.drive.goTo(0, 0, 0);
    }

    protected void interrupted() {
    }
}
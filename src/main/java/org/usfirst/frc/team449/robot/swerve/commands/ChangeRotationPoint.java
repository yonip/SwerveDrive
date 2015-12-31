package org.usfirst.frc.team449.robot.swerve.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team449.robot.Robot;

import java.awt.Point;

/**
 * Changes the point/axis around which the robot will be rotating around to the one passed in the constructor.
 * The point is defined such that the robot is at the origin, with the front in positive y and the right in positive x.
 * Units are currently in meters
 * This command only requires the initialize method to run and then it is done immediately. No other method must be implemented.
 */
public class ChangeRotationPoint extends Command {
    private final Point point;

    /**
     * Creates a new command that, when initialized, sets the point around which the robot is rotating to the given point.
     * The point is defined such that (0,0) is the center of the robot, (1,0) is one meter to the right and (0,1) is one meter forward
     * @param p the point around which to rotate
     */
    public ChangeRotationPoint(final Point p) {
        requires(Robot.drive);
        this.point = p;
    }

    protected void initialize() {
        Robot.drive.setRotationPoint(point);
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return true;
    }

    protected void end() {
    }

    protected void interrupted() {
    }
}
package org.usfirst.frc.team449.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public final Joystick driveSwerveJoystick;
    public final Joystick driveSwerveRotationJoystick;

    public OI() {
        System.out.println("OI init started");
        driveSwerveJoystick = new Joystick(OIMap.DRIVE_SWERVE_JOYSTICK);
        driveSwerveRotationJoystick = new Joystick(OIMap.DRIVE_SWERVE_ROTATION_JOYSTICK);

        System.out.println("OI init ended");
    }

    /**
     * gets the y component of the vector of the swerve joystick (also forward-back or pitch)
     * @return value from -1 to 1
     */
    public double getSwerveStickY() {
        return getJoystickAxisY(driveSwerveJoystick);
    }

    /**
     * gets the y component of the vector of the swerve joystick (also left-right or roll)
     * positive is
     * @return value from -1 to 1
     */
    public double getSwerveStickX() {
        return getJoystickAxisX(driveSwerveJoystick);
    }

    /**
     * gets the rotation value from the rotation joystick (also z axis, twist or yaw)
     * positive is
     * @return value from -1 to 1
     */
    public double getSwerveStickTheta() {
        return getJoystickAxisX(driveSwerveRotationJoystick);
    }

    public double getJoystickAxisX(Joystick joystick)
    {
        return joystick.getAxis(Joystick.AxisType.kX);
    }

    public double getJoystickAxisY(Joystick joystick)
    {
        return joystick.getAxis(Joystick.AxisType.kY);
    }

    public double getJoystickAxisThrottle(Joystick joystick)
    {
        return joystick.getAxis(Joystick.AxisType.kThrottle);
    }

    public double getJoystickAxisZ(Joystick joystick)
    {
        return joystick.getAxis(Joystick.AxisType.kZ);
    }


}


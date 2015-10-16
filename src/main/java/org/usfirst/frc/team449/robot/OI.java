package org.usfirst.frc.team449.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public final Joystick driveSwerveJoystick;

	public OI() {
		System.out.println("OI init started");
		driveSwerveJoystick = new Joystick(RobotMap.DRIVE_SWERVE_JOYSTICK);

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
	 * gets the yaw value from the swerve joystick (also z axis rotation, twist or yaw)
	 * positive is
	 * @return value from -1 to 1
	 */
	public double getSwerveStickTheta() {
		return getJoystickAxisZ(driveSwerveJoystick);
	}

	/**
	 * 
	 * @param joystick
	 * @return
	 */
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
	
	
}//end class


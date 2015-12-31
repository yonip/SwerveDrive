package org.usfirst.frc.team449.robot;

/**
 * The OIMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class OIMap {
    /**
     * Joystick usb port for Joystick 0
     */
    public static final int JOYSTICK_0 = 0;

    /**
     * Joystick usb port for Joystick 1
     */
    public static final int JOYSTICK_1 = 1;

    /**
     * Joystick usb port for Joystick 2
     */
    public static final int JOYSTICK_2 = 2;

    /**
     * Joystick usb port for Joystick 3
     */
    public static final int JOYSTICK_3 = 3;


    /**
     * Joystick for swerve control
     */
    public static final int DRIVE_SWERVE_JOYSTICK = JOYSTICK_0;

    /**
     * Joystick for swerve rotation
     */
    public static final int DRIVE_SWERVE_ROTATION_JOYSTICK = JOYSTICK_1;
}//end class

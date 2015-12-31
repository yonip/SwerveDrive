package org.usfirst.frc.team449.robot.swerve;

/**
 * The OIMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 * TODO Make all of the values real ones
 */
public class SwerveMap {
    public static class Motors {
        public static class Linear {
            public static final int FRONT_LEFT = -1;
            public static final int FRONT_RIGHT = -1;
            public static final int BACK_LEFT = -1;
            public static final int BACK_RIGHT = -1;
        }
        public static class Angular {
            /**
             * the maximum velocity in rad/sec of the rotation motors when controlled manually
             */
            public static final double DEFAULT_MAX_VELOCITY = -1;
            public static final int FRONT_LEFT = -1;
            public static final int FRONT_RIGHT = -1;
            public static final int BACK_LEFT = -1;
            public static final int BACK_RIGHT = -1;
        }
    }

    public static class Encoders {

        /* Motor controller channels. */
        public static class Linear {
            public static final int CPR = -1;
            public static final int FRONT_LEFT_A = -1;
            public static final int FRONT_LEFT_B = -1;
            public static final int FRONT_RIGHT_A = -1;
            public static final int FRONT_RIGHT_B = -1;
            public static final int BACK_LEFT_A = -1;
            public static final int BACK_LEFT_B = -1;
            public static final int BACK_RIGHT_A = -1;
            public static final int BACK_RIGHT_B = -1;
        }

        public static class Angular {
            public static final int CPR = -1;
            public static final int FRONT_LEFT_A = -1;
            public static final int FRONT_LEFT_B = -1;
            public static final int FRONT_RIGHT_A = -1;
            public static final int FRONT_RIGHT_B = -1;
            public static final int BACK_LEFT_A = -1;
            public static final int BACK_LEFT_B = -1;
            public static final int BACK_RIGHT_A = -1;
            public static final int BACK_RIGHT_B = -1;

        }
    }

    /**
     * DriveSwerve PID P term
     */
    public static final double P = 0.005;

    /**
     * DriveSwerve PID I term
     */
    public static final double I = 0;

    /**
     * DriveSwerve PID D term
     */
    public static final double D = 0;

    /**
     * DriveSwerve PID F term for speed control
     */
    public static final double F = 0;

    /**
     * PID tolerance
     */
    public static final double TOLERANCE = -1;

    public static final double WHEEL_DIAMETER = 4;

    /**
     * Hard limit for DriveSwerve speed under PID control in Rotations per second
     */
    public static final int MAX_RATE = 4;
    /**
     * whether by default velocity is controlled manually or not (if not then PID)
     */
    public static final boolean DEFAULT_VELOCITY_MANUAL = true;
    /**
     * whether by default rotation is controlled manually or not (if not then PID)
     */
    public static final boolean DEFAULT_ROTATION_MANUAL = false;
    /**
     * distance between the center of the robot and each side, in meters
     * should also be the magnitude of the "x coordinate" of all the swerve modules
     */
    public static final double HALF_WIDTH = -1;
    /**
     * distance between the center of the robot and the front and back, in meters
     * should also be the magnitude of the "y coordinate" of all the swerve modules
     */
    public static final double HALF_LENGTH = -1;
    /**
     * factor to convert a value between -1 and 1 a velocity in meters per second
     * in practice, this is the default maximum possible linear velocity for the whole robot
     */
    public static final double DEFAULT_MAX_LINEAR_VELOCITY = -1;
    /**
     * factor to convert a value between -1 and 1 a velocity in radians per second
     * in practice, this is the default maximum possible angular velocity for the whole robot
     */
    public static final double DEFAULT_MAX_ANGULAR_VELOCITY = -1;
}

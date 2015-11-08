package org.usfirst.frc.team449.robot.swerve;

/**
 * The OIMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class SwerveMap {
	
	//============================DriveSwerve Constants=======================
	public static class Motors {
		public static class Velocity {
			/**
		 	* motor controller channel for the front left drive motor
		 	*/
			public static final int FRONT_LEFT = -1;

			/**
			 * motor controller channel for the front right drive motor
			 */
			public static final int FRONT_RIGHT = -1;

			/**
			 * motor controller channel for the back left drive motor
			 */
			public static final int BACK_LEFT = -1;

			/**
			 * motor controller channel for the back right drive motor
			 */
			public static final int BACK_RIGHT = -1;


		}
		public static class Rotater {
			/**
			 * motor controller channel for the front left rotation motor
			 */
			public static final int FRONT_LEFT = -1;

			/**
			 * motor controller channel for the front right rotation motor
			 */
			public static final int FRONT_RIGHT = -1;

			/**
			 * motor controller channel for the back left rotation motor
			 */
			public static final int BACK_LEFT = -1;

			/**
			 * motor controller channel for the back right rotation motor
			 */
			public static final int BACK_RIGHT = -1;
		}
	}

	public static class Encoders {
		public static final int CPR = 256;

		/* Motor controller channels. */
		public static class Velocity {
			public static final int FRONT_LEFT_A = -1;
			public static final int FRONT_LEFT_B = -1;
			public static final int FRONT_RIGHT_A = -1;
			public static final int FRONT_RIGHT_B = -1;
			public static final int BACK_LEFT_A = -1;
			public static final int BACK_LEFT_B = -1;
			public static final int BACK_RIGHT_A = -1;
			public static final int BACK_RIGHT_B = -1;
		}

		public static class Rotater {
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
	 * Hard limit for DriveSwerve speed under PID control in Rotations per second
	 */
	public static final int MAX_RATE = 4;
	
	/**
	 * whether default drive is manual or not (if not then PID)
	 */
	public static final boolean DEFAULT_MANUAL = true;
}

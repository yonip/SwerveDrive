package org.usfirst.frc.team449.robot.swerve;

import org.usfirst.frc.team449.robot.OIMap;
import org.usfirst.frc.team449.robot.swerve.commands.DriveSwerveRobot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team449.robot.swerve.components.MotorCluster;

/**
 * The DriveSwerve subsystem.
 */
public class DriveSwerve extends Subsystem {
    
	//drive PID motor systems
	/**
	 * SwerveModule that controls the speed and rotation of the front-left wheel
	 */
	private final SwerveModule frontLeftMotors;
	/**
	 * SwerveModule that controls the speed and rotation of the front-right wheel
	 */
	private final SwerveModule frontRightMotors;
	/**
	 * SwerveModule that controls the speed and rotation of the front-left wheel
	 */
	private final SwerveModule backLeftMotors;
	/**
	 * SwerveModule that controls the speed and rotation of the front-right wheel
	 */
	private final SwerveModule backRightMotors;

	/**
	 * diameter of the wheels on the robot
	 */
	private final double wheelDiameter;
	
	private final int maxRate;
	
	private boolean isManual;
	
	/**
	 * control mode manual - drivers in direct control
	 */
	public static final int MANUAL 	= 0;
	
	/**
	 * control mode PID - PID system is in control
	 */
	public static final int PID		= 1;
	
	/**
	 * Initialize the DriveSwerve subsystem
	 */
	public DriveSwerve(){
		System.out.println("DriveSwerve init started");

		//initialize motor clusters and add slaves
		this.frontLeftMotors = generateModule(SwerveMap.Motors.Velocity.FRONT_LEFT, SwerveMap.Encoders.Velocity.FRONT_LEFT_A,
				SwerveMap.Encoders.Velocity.FRONT_LEFT_B, SwerveMap.Motors.Rotater.FRONT_LEFT, SwerveMap.Encoders.Rotater.FRONT_LEFT_A,
				SwerveMap.Encoders.Rotater.FRONT_LEFT_B);
		this.leftMotors = new MotorCluster(new VictorSP(OIMap.DRIVE_L1)); 	//first motor
		this.leftMotors.addSlave(new VictorSP(OIMap.DRIVE_L2));				//attach second motor
		
		this.rightMotors = new MotorCluster(new VictorSP(OIMap.DRIVE_R1)); 	//first motor
		this.rightMotors.addSlave(new VictorSP(OIMap.DRIVE_R2));				//attach second motor
		
		this.leftEncoder 	= new Encoder(OIMap.DRIVE_ENCODER_LA, OIMap.DRIVE_ENCODER_LB, false);
		this.rightEncoder	= new Encoder(OIMap.DRIVE_ENCODER_RA, OIMap.DRIVE_ENCODER_RB, false);
		
		this.wheelDiameter = 4*Math.PI;
		this.leftEncoder.setDistancePerPulse(wheelDiameter/ OIMap.DRIVE_ENCODER_CPR);
		this.rightEncoder.setDistancePerPulse(-wheelDiameter/ OIMap.DRIVE_ENCODER_CPR);	//negated because mirrored
		
		
		this.leftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		this.rightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);

		this.leftController = new PIDController(OIMap.DRIVE_P, OIMap.DRIVE_I, OIMap.DRIVE_D, OIMap.DRIVE_F, leftEncoder, this.leftMotors);
		this.rightController = new PIDController(OIMap.DRIVE_P, OIMap.DRIVE_I, OIMap.DRIVE_D, OIMap.DRIVE_F, rightEncoder, this.rightMotors);
		
		this.maxRate = OIMap.DRIVE_MAX_RATE;

		this.setManual(OIMap.DRIVE_DEFAULT_MANUAL);
		System.out.println("DriveSwerve init finished");
	}//end drive
	
	/**
	 * Sends power to the two left and right two motors on the drive frame. If in PID control, it is fraction of max absolute speed set in OIMap.
	 * If in Manual, it is fraction of maximum power.
	 * @param leftPower - The fraction of power to supply to the two left motors, from -1 to 1
	 * @param rightPower - The fraction of power to supply to the two right motors, from -1 to 1
	 */
	public void setThrottle(double leftPower, double rightPower){

		rightPower = -rightPower;	//negated because left motors clusters are reversed
		
		if(this.isManual())
		{
			this.leftMotors.set(leftPower);
			this.rightMotors.set(rightPower);
		} else { // PID
			this.leftController.setSetpoint(this.maxRate * leftPower);
			this.rightController.setSetpoint(this.maxRate * rightPower);
		}

	}//end move()

	/**
	 * @return the current rate of the left side wheels in RPS
	 */
	public double getLeftVel(){
		return -this.leftEncoder.getRate();
	}

	/**
	 * @return the current rate of the right side wheels in RPS
	 */
	public double getRightVel(){
		return this.rightEncoder.getRate();
	}
	
	/**
	 * 
	 * @return the current displacement of the left wheels
	 */
	public double getLeftDis(){
		return -this.leftEncoder.getDistance();
	}
	
	/**
	 * 
	 * @return the current displacement of the right wheels
	 */
	public double getRightDis(){
		return this.rightEncoder.getDistance();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());

    	setDefaultCommand(new DriveSwerveRobot());
    }
    

    /**
     * toggle the control mode
     */
    public void toggleControlMode()
    {
    	this.setManual(!this.isManual());
    }
    
    /**
     * set the control mode of this
     * @param isOn is manual on or off (if off, then PID)
     */
    public void setManual(boolean isOn)
    {
    	if(isOn)
    	{
    		this.leftController.disable();
    		this.rightController.disable();
    	} else {
    		this.leftController.enable();
    		this.rightController.enable();
    	}
    	
    	this.isManual = isOn;
    }
    
    /**
     * returns the mode the drive is in
     * @return DriveSwerve.MANUAL or DriveSwerve.PID
     */
    public boolean isManual()
    {
    	return this.isManual;
    }

	protected static SwerveModule generateModule(int velocityMotorPort, int velocityEncoderPortA, int velocityEncoderPortB, int rotationMotorPort, int rotationEncoderPortA, int rotationEncoderPortB) {
		return new SwerveModule(new VictorSP(velocityMotorPort), new Encoder(velocityEncoderPortA, velocityEncoderPortB), new VictorSP(rotationMotorPort), new Encoder(rotationEncoderPortA, rotationEncoderPortB));
	}
}//end class


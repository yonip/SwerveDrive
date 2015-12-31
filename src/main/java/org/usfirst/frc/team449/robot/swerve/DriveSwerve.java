package org.usfirst.frc.team449.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SerialPort;
import org.usfirst.frc.team449.robot.swerve.commands.DriveSwerveRobot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.awt.*;

/**
 * The DriveSwerve subsystem.
 */
public class DriveSwerve extends Subsystem {
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
     * gyro to account for rotation of robot relative to field
     * currently assume robot starts at the same direction as the field
     */
    private final AHRS gyro;
    /**
     * the point around which the robot should rotate, relative to the robot
     * The point is defined such that (0,0) is the center of the robot, (1,0) is one meter to the right and (0,1) is one meter forward
     */
    private Point rotationPoint;
    /**
     * the last state the modules were set to for velocity
     * true for manual, false for PID
     */
    private boolean lastManualVelocity;
    /**
     * the last state the modules were set to for rotation
     * true for manual, false for PID
     */
    private boolean lastManualRotation;

    /**
     * Initialize the DriveSwerve subsystem
     */
    public DriveSwerve(){
        System.out.println("DriveSwerve init started");

        this.frontLeftMotors = generateModule(SwerveMap.Motors.Linear.FRONT_LEFT, SwerveMap.Encoders.Linear.FRONT_LEFT_A,
                SwerveMap.Encoders.Linear.FRONT_LEFT_B, SwerveMap.Motors.Angular.FRONT_LEFT, SwerveMap.Encoders.Angular.FRONT_LEFT_A,
                SwerveMap.Encoders.Angular.FRONT_LEFT_B);
        this.frontRightMotors = generateModule(SwerveMap.Motors.Linear.FRONT_RIGHT, SwerveMap.Encoders.Linear.FRONT_RIGHT_A,
                SwerveMap.Encoders.Linear.FRONT_RIGHT_B, SwerveMap.Motors.Angular.FRONT_RIGHT, SwerveMap.Encoders.Angular.FRONT_RIGHT_A,
                SwerveMap.Encoders.Angular.FRONT_RIGHT_B);
        this.backLeftMotors = generateModule(SwerveMap.Motors.Linear.BACK_LEFT, SwerveMap.Encoders.Linear.BACK_LEFT_A,
                SwerveMap.Encoders.Linear.BACK_LEFT_B, SwerveMap.Motors.Angular.BACK_LEFT, SwerveMap.Encoders.Angular.BACK_LEFT_A,
                SwerveMap.Encoders.Angular.BACK_LEFT_B);
        this.backRightMotors = generateModule(SwerveMap.Motors.Linear.BACK_RIGHT, SwerveMap.Encoders.Linear.BACK_RIGHT_A,
                SwerveMap.Encoders.Linear.BACK_RIGHT_B, SwerveMap.Motors.Angular.BACK_RIGHT, SwerveMap.Encoders.Angular.BACK_RIGHT_A,
                SwerveMap.Encoders.Angular.BACK_RIGHT_B);

        this.lastManualVelocity = SwerveMap.DEFAULT_VELOCITY_MANUAL;
        this.lastManualRotation = SwerveMap.DEFAULT_ROTATION_MANUAL;

        this.gyro = new AHRS(SerialPort.Port.kMXP);
        this.gyro.reset();
        System.out.println("DriveSwerve init finished");
    }//end drive

    /**
     * any initialization of the subsystem
     * used for resetting gyros, encoders, etc
     */
    public void init() {
        this.gyro.reset();
        this.frontLeftMotors.reset();
        this.frontRightMotors.reset();
        this.backLeftMotors.reset();
        this.backRightMotors.reset();
    }

    /**
     * sets the heading and rotation of the robot, field centric
     * rotation will be around an arbitrary point stored in this class (rotationPoint)
     * @param magnitude the magnitude of the vector for the robot to move in
     * @param angleRadians the angle of the vector for the robot to move in (radians)
     * @param rotationRadians how fast the robot should rotate in radians/second
     */
    public void goTo(double magnitude, double angleRadians, double rotationRadians) {
        //account for angle of robot relative to field
        double theta = angleRadians - this.getAngle();
        //put that back into components because later calculations are done using components
        double yVelocity = Math.sin(theta)*magnitude;
        double xVelocity = Math.cos(theta)*magnitude;

        //now give these values to the modules
        passTo(frontLeftMotors, -SwerveMap.HALF_WIDTH, SwerveMap.HALF_LENGTH, xVelocity, yVelocity, rotationRadians, rotationPoint);
        passTo(frontRightMotors, SwerveMap.HALF_WIDTH, SwerveMap.HALF_LENGTH, xVelocity, yVelocity, rotationRadians, rotationPoint);
        passTo(backLeftMotors, -SwerveMap.HALF_WIDTH, -SwerveMap.HALF_LENGTH, xVelocity, yVelocity, rotationRadians, rotationPoint);
        passTo(backRightMotors, SwerveMap.HALF_WIDTH, -SwerveMap.HALF_LENGTH, xVelocity, yVelocity, rotationRadians, rotationPoint);
    }

    /**
     * the point around which the robot should rotate, relative to the robot
     * The point is defined such that (0,0) is the center of the robot, (1,0) is one meter to the right and (0,1) is one meter forward
     * @param p the point around which to rotate, relative to the robot
     */
    public void setRotationPoint(Point p) {
        this.rotationPoint = p;
    }

    /**
     * the point around which the robot will rotate, relative to the robot
     * The point is defined such that (0,0) is the center of the robot, (1,0) is one meter to the right and (0,1) is one meter forward
     * @return the point around which the robot will rotate, relative to the robot
     */
    public Point getRotationPoint() {
        return this.rotationPoint;
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveSwerveRobot());
    }

    /**
     * toggles the control mode for velocity (manual or PID)
     */
    public void toggleModeVelocity() {
        this.setManualVelocity(!this.isManualVelocity());
    }

    /**
     * toggles the control mode for rotation (manual or PID)
     */
    public void toggleModeRotation() {
        this.setManualRotation(!this.isManualRotation());
    }

    /**
     * whether the velocity should be controlled manually (PID otherwise)
     * @param isOn true if velocity of all modules should be controlled manually, and false otherwise
     */
    public void setManualVelocity(boolean isOn) {
        frontLeftMotors.setManualVelocity(isOn);
        frontRightMotors.setManualVelocity(isOn);
        backLeftMotors.setManualVelocity(isOn);
        backRightMotors.setManualVelocity(isOn);
        this.lastManualVelocity = isOn;
    }

    /**
     * whether the rotation should be controlled manually (PID otherwise)
     * @param isOn true if rotation of all modules should be controlled manually, and false otherwise
     */
    public void setManualRotation(boolean isOn) {
        frontLeftMotors.setManualRotation(isOn);
        frontRightMotors.setManualRotation(isOn);
        backLeftMotors.setManualRotation(isOn);
        backRightMotors.setManualRotation(isOn);
        this.lastManualRotation = isOn;
    }

    /**
     * returns whether the velocity of the modules is controlled manually (otherwise, it's controlled via PID)
     * @return true if the velocity of the modules is controlled manually, and false otherwise
     */
    public boolean isManualVelocity()
    {
        boolean and = frontRightMotors.isManualVelocity() && frontLeftMotors.isManualVelocity() && backRightMotors.isManualVelocity() && backLeftMotors.isManualVelocity();
        boolean or = frontRightMotors.isManualVelocity() || frontLeftMotors.isManualVelocity() || backRightMotors.isManualVelocity() || backLeftMotors.isManualVelocity();
        if(and != or) {
            System.err.println("Manual values for velocity don't correspond! Saying " + this.lastManualVelocity + " anyway");
        }
        return this.lastManualVelocity;
    }

    /**
     * returns whether the rotation of the modules is controlled manually (otherwise, it's controlled via PID)
     * @return true if the rotation of the modules is controlled manually, and false otherwise
     */
    public boolean isManualRotation()
    {
        boolean and = frontRightMotors.isManualRotation() && frontLeftMotors.isManualRotation() && backRightMotors.isManualRotation() && backLeftMotors.isManualRotation();
        boolean or = frontRightMotors.isManualRotation() || frontLeftMotors.isManualRotation() || backRightMotors.isManualRotation() || backLeftMotors.isManualRotation();
        if(and != or) {
            System.err.println("Manual values for rotation don't correspond! Saying " + this.lastManualRotation + " anyway");
        }
        return this.lastManualRotation;
    }

    /**
     * gets the current angle of the whole robot in radians relative to where the gyro was last reset (likely initial position)
     * uses the modulo (%) operation to wrap around and keep sign
     * @return a value between -PI and PI, representing the current wheel's rotation
     */
    public double getAngle() {
        return this.gyro.getAngle() % Math.PI;
    }

    /**
     * takes the ports for the motors and encoders and return an initialized SwerveModule with motors and encoders at the corresponding ports
     * encoders have their DPP and PIDSourceParameter set
     * @param velocityMotorPort port for the velocity motor
     * @param velocityEncoderPortA port A of the encoder for velocity
     * @param velocityEncoderPortB port b of the encoder for velocity
     * @param rotationMotorPort port for the rotation motor
     * @param rotationEncoderPortA port A of the encoder for rotation
     * @param rotationEncoderPortB port B of the encoder for rotation
     * @return a SwerveModule object with encoders and motors set using the given ports
     */
    protected static SwerveModule generateModule(int velocityMotorPort, int velocityEncoderPortA, int velocityEncoderPortB, int rotationMotorPort, int rotationEncoderPortA, int rotationEncoderPortB) {
        Encoder velEnc = new Encoder(velocityEncoderPortA, velocityEncoderPortB);
        Encoder rotEnc = new Encoder(rotationEncoderPortA, rotationEncoderPortB);

        velEnc.setDistancePerPulse(SwerveMap.WHEEL_DIAMETER / SwerveMap.Encoders.Linear.CPR);
        rotEnc.setDistancePerPulse(1.0 / SwerveMap.Encoders.Angular.CPR); // because we just want it to go to radians, so distance should equal angle

        velEnc.setPIDSourceParameter(PIDSourceParameter.kRate);
        rotEnc.setPIDSourceParameter(PIDSourceParameter.kRate);

        return new SwerveModule(new VictorSP(velocityMotorPort), velEnc, SwerveMap.DEFAULT_VELOCITY_MANUAL, new VictorSP(rotationMotorPort), rotEnc, SwerveMap.DEFAULT_ROTATION_MANUAL);
    }

    /**
     * calculates the angle and velocity for the given module based on the given paramaters.
     * @param module the module to assign velocity and rotation
     * @param moduleX the x-coordinate of the module, where right is positive, relative to the center of the robot
     * @param moduleY the y-coordinate of the module, where forward is positive, relative to the center of the robot
     * @param xVelocity the x component of the velocity vector for the robot
     * @param yVelocity the y component of the velocity vector for the robot
     * @param rotationRadians the rate of rotation of the whole robot, in radians per second
     * @param rotationPoint the point around which the robot will rotate, where (0,0) is the origin, forward si positive y and right is positive x
     */
    protected static void passTo(SwerveModule module, double moduleX, double moduleY, double xVelocity, double yVelocity, double rotationRadians, Point rotationPoint) {
        // to store the components of the rotation vector for each module
        double deltaX, deltaY;
        // the final components of the velocity vector for the module
        double finalVelX, finalVelY;
        // the actual variables that will be passed to the module
        double magnitude, angleDegrees;
        // components scaled according to their distance from the center for angular -> linear velocity
        // yes, this is correct
        deltaX = moduleY - rotationPoint.getY();
        deltaY = moduleX - rotationPoint.getX();
        // first part for the respective component of the vector rotating, and the second for the vector translating
        finalVelX = deltaX*rotationRadians + xVelocity;
        finalVelY = deltaY*rotationRadians + yVelocity;
        magnitude = Math.sqrt(finalVelX * finalVelX + finalVelY * finalVelY);
        angleDegrees = Math.toDegrees(Math.atan2(finalVelY, finalVelX));
        module.goTo(magnitude, angleDegrees);
    }
}//end class


package org.usfirst.frc.team449.robot.swerve.components;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class MotorCluster extends Subsystem implements SpeedController {

	/**
	 * an ArrayList of all the motors in this cluster
	 */
	private final SpeedController[] controllers;
	private final boolean[] controllerFlags; //add option to invert motor input

	/**
	 * the previous value that the motors were set to
	 */
	private double lastSet = 0;

	/**
	 * Constructs a MotorCluster with a single motor
	 * @param controllers all the motors for the cluster
	 * @param flags the invert flags for the motor
	 */
	public MotorCluster(SpeedController[] controllers, boolean[] flags)
	{
		this.controllers = controllers;
		this.controllerFlags = flags;

		if(this.controllers.length != this.controllerFlags.length) {
			System.out.println("Flags and controllers aren't the same length. ignoring flags.");
		}
		
		this.lastSet = 0;
	}

	@Override
	public void pidWrite(double output) {
		this.set(output);
	}

	@Override
	public double get() {
		return this.lastSet;
	}

	@Override
	public void set(double speed, byte syncGroup) {
		System.out.println("This shit is deprecated and shouldn't be called.\n TL;DR: YOU DONE FUCKED UP");
	}

	@Override
	public void set(double speed) {

		if(controllers.length != controllerFlags.length) // if for some reason the invert flag array got screwed up, ignore it
		{
			System.err.println("invert flags don't match with slaves");
			for(int i=0; i < controllers.length; i++)
			{
				//System.out.println("PID setting throttle " + output);
				controllers[i].set(speed);
			}

			return;
		}
		else // invert flags and slave lists all bueno
		{
			for(int i=0; i < controllers.length; i++)
			{
				//System.out.println("setting throttle " + output);
				if(controllerFlags[i]) // if invert flag, invert slave motor output
				{
					controllers[i].set(-speed);
				}
				else // if slave is not to be inverted
				{
					controllers[i].set(speed);
				}
			}
		}
		
		this.lastSet = speed;
	}

	@Override
	public void disable() {
		
		for(int i=0; i < this.controllers.length; i++)
			controllers[i].disable();
	}

	@Override
	protected void initDefaultCommand() {
		//no default command yo
	}
}//end class
package frc.robot.nerdyfiles.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 *
 */
public class NerdyFlightStick extends Joystick {

    public NerdyFlightStick(int port) {
        super(port);
    }

	/*
		AXIS:
		#	Description		Direction   			Positive
		--	---------------	---------------------	--------
		0	Joystick tilt	Right/Left				Right
		1	Joystick tilt	Forward/back			Back (negated in this class getter)
		2	Throttle tilt	Forward/Back			Back (negated in this class getter)
		3	Joystick rotate	Right/Left (Rotation)	Right
		4	Throttle rocker	Right/Left (Rocker)		Right
	 */

	public JoystickButton			RightTrigger				= new JoystickButton(this, 1);	//The digital trigger on the back of the joystick
	public JoystickButton			StripedButton				= new JoystickButton(this, 2);	//The orange and black striped button on joystick
	public JoystickButton			RightKnuckleButton			= new JoystickButton(this, 3);	//The button on the top-right of the joytstick
	public JoystickButton			L3Button					= new JoystickButton(this, 4);	//The button on the front right of the joystick
	
	public JoystickButton			ThrottleTopThumbButton		= new JoystickButton(this, 5);	//The highest button of the throttle's thumb buttons
	public JoystickButton			ThrottleMidThumbButton		= new JoystickButton(this, 6);	//The middle button of the throttle's thumb buttons
	public JoystickButton			ThrottleBottomThumbButton	= new JoystickButton(this, 7);	//The lowest button of the throttle's thumb buttons
	
	public JoystickButton			PalmButton					= new JoystickButton(this, 8);	//The button on the palmrest of the throttle
	public JoystickButton			TopIndexButton   			= new JoystickButton(this, 9);	//The higher button on the back right of the throttle
	public JoystickButton			BottomIndexButton			= new JoystickButton(this, 10);	//The lower button on the back right of the throttle
	
	public JoystickButton			SEButton					= new JoystickButton(this, 11); //The "SE" button on the throttle
	public JoystickButton			STButton					= new JoystickButton(this, 12); //The "ST" button on the throttle
	
	public JoystickPOVButton		povUp			        	= new JoystickPOVButton(this, 0);
	public JoystickPOVButton		povUpRight      			= new JoystickPOVButton(this, 45);
	public JoystickPOVButton		povRight		        	= new JoystickPOVButton(this, 90);
	public JoystickPOVButton		povDownRight        		= new JoystickPOVButton(this, 135);
	public JoystickPOVButton		povDown		        	    = new JoystickPOVButton(this, 180);
	public JoystickPOVButton		povLeft			            = new JoystickPOVButton(this, 270);
	public JoystickPOVButton		povUpLeft		        	= new JoystickPOVButton(this, 315);


    public double getStickY() {
        return this.getRawAxis(0);
    }

    public double getStickX() {
        return -this.getRawAxis(1);
    }

    public double getThrottle() {
        return -this.getRawAxis(2);
    }
    
    public double getStickTwist() {
        return this.getRawAxis(3);
    }
    
    public double getThrottleToggle() {
        return this.getRawAxis(4);
    }

}
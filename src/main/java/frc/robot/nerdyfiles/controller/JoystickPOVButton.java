package frc.robot.nerdyfiles.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * Trigger commands from an POV button on a joystick. POV button angles range
 * from 0 to 315 degrees, with 0 being up and continuing clockwise in
 * increments of 45 degrees. For example, left equals 270 degrees, bottom
 * right equals 135 degrees, etc.
 * 
 * @author rhys.barrie@team2337.com
 *
 */

/*
 * The following example code placed in the OI class turns the up and down POV buttons into buttons:
 * ----------------------------------------------------------------------------
 * //Create a POVButton for each POV direction
 * Joystick joystick = new Joystick(0);
 * public JoystickPOVButton POV_Up = new JoystickAnalogButton(joystick, 0),
 * 							   POV_Dn = new JoystickAnalogButton(joystick, 180);
 * 
 * //Link the buttons to commands
 * POV_Up.whenPressed(new ExampleCommand1());
 * POV_Dn.whenPressed(new ExampleCommand2());
 */

public class JoystickPOVButton extends Button {

	GenericHID m_joystick;
	private double angle;

	/**
	 * Create a button for triggering commands off a joystick's POV buttons.
	 * 
	 * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc).
	 * @param angle The angle of the POV button to watch.
	 */
	public JoystickPOVButton(GenericHID joystick, double angle) {
		m_joystick = joystick;
		this.angle = angle;
	}
 
	/**
	 * Get the defined POV angle to watch.
	 * @return The angle value.
	 */
	public double getAngle(){
		return angle;
	}
	

	public boolean get() {
		return m_joystick.getPOV() == angle;
	}

}
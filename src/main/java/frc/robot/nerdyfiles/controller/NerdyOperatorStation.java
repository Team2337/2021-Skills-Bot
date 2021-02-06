package frc.robot.nerdyfiles.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 *
 */
public class NerdyOperatorStation extends Joystick {

    public NerdyOperatorStation(int port) {
        super(port);
    }

    /*
     * --- DRIVER STATION CONTROLS ---
     * 
     * These button/switch names and ports will need to be updated each year if the
     * driver's station is redesigned.
     *
     */
    public JoystickButton  GreenButton  = new JoystickButton(this, 1);
    public JoystickButton  RedButton    = new JoystickButton(this, 2);
    public JoystickButton  WhiteButton  = new JoystickButton(this, 7);
    public JoystickButton  YellowButton = new JoystickButton(this, 8);
    public JoystickButton  BlueButton   = new JoystickButton(this, 9);
    public JoystickButton  BlackButton  = new JoystickButton(this, 10);

    public JoystickButton  ClearSwitch  = new JoystickButton(this, 3);
    public JoystickButton  YellowSwitch = new JoystickButton(this, 4);
    public JoystickButton  BlueSwitch   = new JoystickButton(this, 5);
    public JoystickButton  BlackSwitch  = new JoystickButton(this, 6);

    public boolean clearSwitchOn() {
        return this.ClearSwitch.get();
    }

    public boolean yellowSwitchOn() {
        return this.YellowSwitch.get();
    }

    public boolean blackSwitchOn() {
        return this.BlackSwitch.get();
    }

    public boolean blueSwitchOn() {
        return this.BlueSwitch.get();
    }
}
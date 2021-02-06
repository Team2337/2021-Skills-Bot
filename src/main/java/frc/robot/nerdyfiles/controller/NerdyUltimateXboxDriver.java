package frc.robot.nerdyfiles.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.hal.HAL;

/**
 * Manages setting up buttons and joysticks for a Razer Wolverine Ultimate Xbox One Controller
 * 
 * @author Robin B. Emily H.
 */
public class NerdyUltimateXboxDriver extends Joystick {
    public NerdyUltimateXboxDriver(int port) {
        super(port);
    }

    public JoystickButton           greenA             = new JoystickButton(this, 1);
    public JoystickButton           redB               = new JoystickButton(this, 2);
    public JoystickButton           blueX              = new JoystickButton(this, 3);
    public JoystickButton           yellowY            = new JoystickButton(this, 4);
    public JoystickButton           bumperLeft         = new JoystickButton(this, 5);
    public JoystickButton           bumperRight        = new JoystickButton(this, 6);
    public JoystickButton           back               = new JoystickButton(this, 7);
    public JoystickButton           start              = new JoystickButton(this, 8);
    public JoystickButton           leftStickButton    = new JoystickButton(this, 9);
    public JoystickButton           rightStickButton   = new JoystickButton(this, 10);
    public JoystickAnalogButton     triggerLeft        = new JoystickAnalogButton(this, 2);
    public JoystickAnalogButton     triggerRight       = new JoystickAnalogButton(this, 3);
    public JoystickPOVButton        povUp              = new JoystickPOVButton(this, 0);
    public JoystickPOVButton        povUpRight         = new JoystickPOVButton(this, 45);
    public JoystickPOVButton        povRight           = new JoystickPOVButton(this, 90);
    public JoystickPOVButton        povDownRight       = new JoystickPOVButton(this, 135);
    public JoystickPOVButton        povDown            = new JoystickPOVButton(this, 180);
    public JoystickPOVButton        povDownLeft        = new JoystickPOVButton(this, 225);
    public JoystickPOVButton        povLeft            = new JoystickPOVButton(this, 270);
    public JoystickPOVButton        povUpLeft          = new JoystickPOVButton(this, 315);

    public JoystickButton           macroOne           = null;
    public JoystickButton           macroTwo           = null;
    public JoystickButton           macroThree         = greenA;
    public JoystickButton           macroFour          = redB;
    public JoystickButton           macroFive          = blueX;
    public JoystickButton           macroSix           = yellowY;
 
    private int m_outputs;
    private short m_leftRumble;
    private short m_rightRumble;

    /**
     * 
     */
    public double getTriggerTwist() {
        double leftTriggerValue = this.getRawAxis(2);
        double rightTriggerValue = -1 * this.getRawAxis(3);

        return leftTriggerValue + rightTriggerValue;
    }
    public double getLeftStickX() {
        return this.getRawAxis(0);
    }
    public double getLeftStickY() {
        return -this.getRawAxis(1);
    }
    public double getLeftTrigger() {
        return -this.getRawAxis(2);
    }
    public double getRightTrigger() {
        return -this.getRawAxis(3);
    }
    public double getRightStickX() {
        return this.getRawAxis(4);
    }
    public double getRightStickY() {
        return -this.getRawAxis(5);
    }

    /**
     * Individually sets the Rumble Speeds on the left and right side of the controller
     * @param leftValue - left rumble speed
     * @param rightValue - right rumble speed
     * <br/>
     * <br/>
     * <strong>NOTE:</strong> Right rumble is more intense than left due to the motors inside the controller
     */
    public void setRumbleSpeed(double leftValue, double rightValue) {
        setRumble(RumbleType.kLeftRumble, leftValue);
        setRumble(RumbleType.kRightRumble, rightValue);
    }

    public void setRumble(RumbleType type, double value) {
        if (value < 0) {
            value = 0;
        } else if (value > 1) {
            value = 1;
        }
        if (type == RumbleType.kLeftRumble) {
            m_leftRumble = (short) (value * 65535);
        } else {
            m_rightRumble = (short) (value * 65535);
        }
        HAL.setJoystickOutputs((byte) getPort(), m_outputs, m_leftRumble, m_rightRumble);
    }

    /**
     * Checks the rumble timer to decide the rumble intensity
     * @param timer - input timer variable (int)
     * @param intensity - intensity of the rumble (0 -> 1.0)
     * @return
     */
    public double setRumbleTimer(int timer, double intensity) {
        return (timer < 100) ? intensity : 0;
    }
}
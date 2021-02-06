package frc.robot;

import frc.robot.commands.swerve.*;
import frc.robot.nerdyfiles.controller.*;
import frc.robot.subsystems.OperatorAngleAdjustment;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Vision;
import frc.robot.commands.vision.SetBallTracking;

/**
 * OI Class where all controllers and button presses are placed
 */
public class OI {
    public NerdyUltimateXboxDriver driverJoystick = new NerdyUltimateXboxDriver(0);
    public NerdyUltimateXboxOperator operatorJoystick = new NerdyUltimateXboxOperator(1);
    public NerdyOperatorStation operatorControls = new NerdyOperatorStation(2);

    public OI(OperatorAngleAdjustment operatorAngleAdjustment, Pigeon pigeon, Vision vision) {
        /* --- DRIVER JOYSTICK --- */

        // Sets the robots rotation angle offset, while the button is being pressed
        driverJoystick.bumperLeft.whenPressed(new ChangeGyroAngleOffset(operatorAngleAdjustment, pigeon, vision, true));
        driverJoystick.bumperLeft.whenReleased(new ChangeGyroAngleOffset(operatorAngleAdjustment, pigeon, vision, false));

        // Slow rotates to the right
        driverJoystick.redB.whenPressed(new SetSlowRotateMode(operatorAngleAdjustment, pigeon, true, -Constants.Swerve.SLOWROTATESPEED));
        driverJoystick.redB.whenReleased(new SetSlowRotateMode(operatorAngleAdjustment, pigeon, false, 0));

        // Slow rotates to the left
        driverJoystick.blueX.whenPressed(new SetSlowRotateMode(operatorAngleAdjustment, pigeon, true, Constants.Swerve.SLOWROTATESPEED));
        driverJoystick.blueX.whenReleased(new SetSlowRotateMode(operatorAngleAdjustment, pigeon, false, 0));

        driverJoystick.povUp.whenPressed(new ResetGyro(pigeon));

        driverJoystick.back.whenPressed(new ChangeVisionAngleOffset(operatorAngleAdjustment, pigeon, true));
        driverJoystick.back.whenReleased(new ChangeVisionAngleOffset(operatorAngleAdjustment, pigeon, false));

        driverJoystick.start.whenPressed(new SetBallTracking(operatorAngleAdjustment, true));
        driverJoystick.start.whenReleased(new SetBallTracking(operatorAngleAdjustment, false));

        /* --- OPERATOR JOYSTICK --- */

        // Buttons to queue the robot's angle offset
        operatorJoystick.yellowY.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "farShot"));
        operatorJoystick.redB.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "nearShot"));
        operatorJoystick.blueX.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "frontTrenchShot"));
        operatorJoystick.greenA.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "frontTrenchRunShot"));

        operatorJoystick.povUp.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "0"));
        operatorJoystick.povRight.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "90"));
        operatorJoystick.povDown.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "180"));
        operatorJoystick.povLeft.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "270"));

        /* --- DRIVER STATION CONTROLS --- */

        operatorControls.BlackSwitch.whenPressed(new SetGyroAngleOffset(operatorAngleAdjustment, "climbing"));

        operatorControls.BlueSwitch.whenPressed(new ChangeVisionAngleOffset(operatorAngleAdjustment, pigeon, false));
    }

}

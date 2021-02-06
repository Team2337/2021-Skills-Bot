package frc.robot.commands.vision;

import frc.robot.subsystems.OperatorAngleAdjustment;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Sets whether or not we're tracking a ball
 */
public class SetBallTracking extends InstantCommand {
    private OperatorAngleAdjustment operatorAngleAdjustment;
    private boolean ballTrackingEnabled;

    /**
     * Sets whether or not we're tracking a ball
     *
     * @param operatorAngleAdjustment The OperatorAngleAdjustment subsystem
     * @param ballTrackingEnabled     Whether or not we're tracking a ball
     */
    public SetBallTracking(OperatorAngleAdjustment operatorAngleAdjustment, boolean ballTrackingEnabled) {
        this.operatorAngleAdjustment = operatorAngleAdjustment;
        addRequirements(operatorAngleAdjustment);

        this.ballTrackingEnabled = ballTrackingEnabled;
    }

    @Override
    public void initialize() {
        operatorAngleAdjustment.setBallTrackingEnabled(ballTrackingEnabled);
    }
}
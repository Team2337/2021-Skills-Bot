package frc.robot.commands.swerve;

import frc.robot.subsystems.OperatorAngleAdjustment;
import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Sets the slow rotation mode and speed for slow rotation.
 * The rotational changes are applied in SwerveDriveCommand
 * @see SwerveDriveCommand
 * @author Bryce G.
 * @category SWERVE
 */
public class SetSlowRotateMode extends InstantCommand {

  private OperatorAngleAdjustment operatorAngleAdjustment;
  private Pigeon pigeon;

  private boolean slowRotateMode;
  private double slowRotateSpeed;

  /**
   * Sets the slow rotation mode and speed for slow rotation.
   * The rotational changes are applied in SwerveDriveCommand
   * @param subsystem
   * @param slowRotateMode
   * @param slowRotateSpeed
   * @see SwerveDriveCommand
   */
  public SetSlowRotateMode(OperatorAngleAdjustment operatorAngleAdjustment, Pigeon pigeon, boolean slowRotateMode, double slowRotateSpeed) {
    this.operatorAngleAdjustment = operatorAngleAdjustment;
    this.pigeon = pigeon;
    addRequirements(operatorAngleAdjustment, pigeon);

    this.slowRotateMode = slowRotateMode;
    this.slowRotateSpeed = slowRotateSpeed;
  }

  @Override
  public void initialize() {
    // Sets the slow rotate mode and slow rotate speed
    operatorAngleAdjustment.setSlowRotateMode(slowRotateMode);
    operatorAngleAdjustment.setSlowRotateSpeed(slowRotateSpeed);
      if(!slowRotateMode) {
        operatorAngleAdjustment.setOffsetAngle(pigeon.getYawMod());
      }
  }

  @Override
  public void end(boolean interrupted) {
  }
}

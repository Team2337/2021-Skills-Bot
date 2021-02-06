package frc.robot.commands.swerve;

import frc.robot.subsystems.OperatorAngleAdjustment;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Sets the robot's future angle offset. This should be on the <b>OPERATOR</b> joystick
 * @see OperatorAngleAdjustment
 * @author Bryce G., Madison J.
 * @category SWERVE
 */
public class SetGyroAngleOffset extends InstantCommand {

  private final OperatorAngleAdjustment m_subsystem;
  private String mode;

  /**
   * Sets the robot's future angle offset. This should be on the <b>DRIVER</b> joystick
   * @param subsystem - OperatorAngleAdjustment Subsystem Object
   * @param mode - String value signifying the rotation mode the robot is in
   */
  public SetGyroAngleOffset(OperatorAngleAdjustment subsystem, String mode) {
    m_subsystem = subsystem;
    addRequirements(subsystem);

    this.mode = mode;
  }

  @Override
  public void initialize() {
    m_subsystem.setFutureOffsetAngle(mode);
  }

  @Override
  public void end(boolean interrupted) {
  }

}
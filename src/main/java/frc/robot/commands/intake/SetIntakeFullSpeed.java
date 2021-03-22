package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;

/**
 * Sets the intake at full speed
 * @author Michael F.
 */
public class SetIntakeFullSpeed extends SetIntakeSpeed {
  /**
   * Sets the intake motor at full speed
   * @param subsystem The Intake subsystem used by this command
   */
  public SetIntakeFullSpeed(Intake subsystem) {
    super(subsystem, 1);
  }
}

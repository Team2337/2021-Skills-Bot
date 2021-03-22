package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;

/**
 * Stops the intake
 * @author Michael F.
 */
public class StopIntake extends SetIntakeSpeed {
  /**
   * Stops the intake motor
   * @param subsystem The Intake subsystem used by this command
   */
  public StopIntake(Intake subsystem) {
    super(subsystem, 0);
  }
}

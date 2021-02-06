package frc.robot.commands.swerve;

import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Resets the gyro to zero
 * @author Bryce G., Madison J.
 * @category SWERVE
 */
public class ResetGyro extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Pigeon Pigeon;
  
  /**
   * Resets the gyro to zero
   * @param Pigeon - Pigeon Subsystem Object
   */
  public ResetGyro(Pigeon pigeon) {
    this.Pigeon = pigeon;
    addRequirements(pigeon);
  }

  @Override
  public void initialize() {
    Pigeon.resetPidgey();
  }

  @Override
  public void end(boolean interrupted) {
  }

}
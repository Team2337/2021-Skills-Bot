package frc.robot.commands.auto;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Zeros the angle encoders
 * @author Madison J.
 * @category AUTON
 */
public class zeroDriveEncoders extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrivetrain m_subsystem;

  /**
   * Zeros the angle encoders
   * @param subsystem The subsystem used by this command.
   */
  public zeroDriveEncoders(final SwerveDrivetrain subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
      // Goes through 4 times and sets the angle encoders to zero
      m_subsystem.zeroAllDriveEncoders();
  }

  @Override
  public void end(final boolean interrupted) {
  }

}
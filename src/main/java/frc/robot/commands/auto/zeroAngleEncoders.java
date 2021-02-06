package frc.robot.commands.auto;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Zeros the angle encoders
 * @author Madison J.
 * @category AUTON
 */
public class zeroAngleEncoders extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrivetrain m_subsystem;

  /**
   * Zeros the angle encoders
   * @param subsystem The subsystem used by this command.
   */
  public zeroAngleEncoders(SwerveDrivetrain subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    for (int i = 0; i < 4; i++) {
      // Goes through 4 times and sets the angle encoders to zero
      m_subsystem.getModule(i).setAngleEncoder(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
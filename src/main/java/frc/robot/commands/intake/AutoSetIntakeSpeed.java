package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Sets the intake speed
 * @author Michael F.
 */
public class AutoSetIntakeSpeed extends CommandBase {
  private final Intake subsystem;
  private double speed;

  /**
   * Sets the intake speed to a certain amount
   * @param subsystem The Intake subsystem used by this command
   * @param speed The speed to set the intake to (a value between -1 and 1)
   */
  public AutoSetIntakeSpeed(Intake subsystem, double speed) {
    this.subsystem = subsystem;
    this.speed = speed;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setIntakeSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

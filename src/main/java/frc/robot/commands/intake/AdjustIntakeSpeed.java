package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Adjusts the intake speed by a set amount.
 * Makes sure the intake speed does not go above +/- 1.
 * @author Michael F.
 */
public class AdjustIntakeSpeed extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake subsystem;
  private double modifier;

  /**
   * Adjusts the intake speed by a certain amount without setting it over +/- 1
   * @param subsystem The Intake subsystem used by this command
   * @param modifier The amount to change the speed by
   */
  public AdjustIntakeSpeed(Intake subsystem, double modifier){
    this.subsystem = subsystem;
    this.modifier = modifier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    //Set the intake speed but make sure it does not go above +/- 1
    subsystem.setIntakeSpeed(
      Math.copySign(
        Math.min(1, Math.abs(subsystem.getIntakeSetSpeed() + modifier)),
        subsystem.getIntakeSetSpeed() + modifier
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
  }
}

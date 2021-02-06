package frc.robot.commands.vision;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;

  /**
   * Switches to the different conigurations of the limelight
   * @author Zayd A. & Madison J.
   * @category VISION
   */
public class LimelightPipeline extends InstantCommand {
  private Vision subsystem;

  /* --- Integers --- */
  private int pipe;

  /**
  * Switches to the different conigurations of the limelight
  * @param subsystem - Vision subsystem object
  * @param pipe - Chamnges to the different configurations
  */
  public LimelightPipeline(Vision subsystem, int pipe) {
    this.subsystem = subsystem;
    addRequirements(subsystem);

    this.pipe = pipe;
  }

  @Override
  public void initialize() {
    subsystem.switchPipeLine(pipe);
  }

}
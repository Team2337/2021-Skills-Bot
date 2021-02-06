package frc.robot.commands.vision;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Drivecam will be turned on
 * @author Zayd A. & Madison J.
 * @category VISION
 */
public class SwitchDriverCam extends InstantCommand {
  private Vision subsystem;

  /**
   * Drivecam will be turned on
   */
  public SwitchDriverCam(Vision subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.switchPipeLine(9);
  }
}
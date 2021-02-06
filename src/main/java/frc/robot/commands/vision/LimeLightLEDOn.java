package frc.robot.commands.vision;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;

  /**
   * Limelight LEDs will turn on
   * <p><br/>Mode 3 is to turn on the LED</p>
   * @author Zayd A. & Madison J.
   * @category VISION
   */
public class LimeLightLEDOn extends InstantCommand {
  private Vision vision;
  /**
  * Limelight LEDs will turn on
  * <p><br/>Mode 3 is to turn on the LED</p>
  */
  public LimeLightLEDOn(Vision vision) {
    this.vision = vision;
    addRequirements(vision);
  }

  @Override
  public void initialize() {
    vision.setLEDMode(3);
  }

}
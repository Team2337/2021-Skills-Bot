package frc.robot.commands.auto;

import java.io.IOException;
import java.util.Optional;

import frc.robot.subsystems.PixyCam2Wire;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.PixyCam2Wire.GalacticSearchPath;
import frc.robot.commands.auto.galacticsearch.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Uses the PixyCam to run an auton based on the position of a ball.
 * @author Michael F., Zach O., and Nicholas S.
 */
public final class GalacticSearch extends CommandBase {

  private PixyCam2Wire pixy;
  private SwerveDrivetrain drivetrain;

  /**
   * Creates the Galactic Search Command.
   *
   * @param pixy The PixyCam used by this command.
   * @param drivetrain The drivetrain for scheduling commands
   */
  public GalacticSearch(PixyCam2Wire pixy, SwerveDrivetrain drivetrain) {
    this.pixy = pixy;
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    //Get PixyCam values
    Optional<GalacticSearchPath> optionalPath = pixy.getPath();
    if (optionalPath.isPresent()) {
      GalacticSearchPath path = optionalPath.get();
      try {
        switch (path) {
          case RED_A:
            CommandScheduler.getInstance().schedule(new GalacticSearchRedA(drivetrain));
            break;
          case RED_B:
            CommandScheduler.getInstance().schedule(new GalacticSearchRedB(drivetrain));
            break;
          case BLUE_A:
            CommandScheduler.getInstance().schedule(new GalacticSearchBlueA(drivetrain));
            break;
          case BLUE_B:
            CommandScheduler.getInstance().schedule(new GalacticSearchBlueA(drivetrain));
            break;
        }
      } catch(IOException e) {
        e.printStackTrace();
      }
    }
  }

}
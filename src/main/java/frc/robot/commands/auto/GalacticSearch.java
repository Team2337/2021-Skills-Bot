package frc.robot.commands.auto;

import java.io.IOException;
import frc.robot.subsystems.PixyCam;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.commands.auto.galacticsearch.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Uses the PixyCam to run an auton based on the position of a ball.
 * @author Michael F., Zach O., and Nicholas S.
 */
public final class GalacticSearch extends InstantCommand {

  private PixyCam pixy;
  private SwerveDrivetrain drivetrain;

  /**
   * Creates the Galactic Search Command.
   *
   * @param pixy The PixyCam used by this command.
   * @param drivetrain The drivetrain for scheduling commands
   */
  public GalacticSearch(PixyCam pixy, SwerveDrivetrain drivetrain) {
    super();
    this.pixy = pixy;
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize(){
    //Get PixyCam values
    int x = pixy.getLargestTargetX();
    
    try {
      //The values used with x represent the x-position of the closest target in the field of view of the camera.
      if (x >= 0 && x < 79) {
        //Red A path
        CommandScheduler.getInstance().schedule(new GalacticSearchRedA(drivetrain));
      } else if(x >= 79 && x < 158) {
        //Red B path
        CommandScheduler.getInstance().schedule(new GalacticSearchRedB(drivetrain));
      } else if(x >= 158 && x < 237) {
        //Blue A path
        CommandScheduler.getInstance().schedule(new GalacticSearchBlueA(drivetrain));
      } else if(x >= 237 && x <= 315) {
        //Blue B path
        CommandScheduler.getInstance().schedule(new GalacticSearchBlueB(drivetrain));
      }
    } catch(IOException e){
      e.printStackTrace();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

}
package frc.robot.commands.swerve;

import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * 
 * 
 */
public class Pacman {

  private SwerveDrivetrain drivetrain;  //TODO why not used???

    double halfRobotPlus = 1.1; //foot ? probably need meters for all
    
    /**
     * @return
     * 
     */
    public void pacmanSlalom(double forward, double strafe, SwerveDrivetrain drivetrain) {
      this.drivetrain = drivetrain;

      double robotX = drivetrain.getPose().getTranslation().getX();
      double robotY = drivetrain.getPose().getTranslation().getY();

      if (robotX < 5+halfRobotPlus & robotY < 5+halfRobotPlus & forward < 0) forward = 0;

      if (robotX < 10-halfRobotPlus & strafe < 0) strafe = 0; 

      if (robotX > 10-halfRobotPlus & robotX < 20+halfRobotPlus){
        if (robotX < 10 & robotY > 5-halfRobotPlus & robotY < 5+halfRobotPlus & forward > 0) forward = 0;
        if (robotX > 20 & robotY > 5-halfRobotPlus & robotY < 5+halfRobotPlus & forward < 0) forward = 0;
        if (robotY > 4.5000 & robotY < 5+halfRobotPlus & strafe < 0) strafe = 0;
        if (robotY < 4.4999 & robotY > 5-halfRobotPlus & strafe > 0) strafe = 0;
        }

      if (robotX > 19.999+halfRobotPlus & robotX < 25-halfRobotPlus) { /*nothing*/  }

      if (robotX > 25-halfRobotPlus & robotX < 25+halfRobotPlus){
        if (robotX < 25 & robotY > 5-halfRobotPlus & robotY < 5+halfRobotPlus & forward > 0) forward = 0;
        if (robotX > 25 & robotY > 5-halfRobotPlus & robotY < 5+halfRobotPlus & forward < 0) forward = 0;
        if (robotY > 5.0001 & robotY < 5+halfRobotPlus & strafe < 0) strafe = 0;
        if (robotY < 5.0000 & robotY > 5-halfRobotPlus & strafe > 0) strafe = 0;
        }
    
    } //end pacmanSlalom.  probably and else's


}
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The code for retrieving information from the PixyCam using analog/digital ports.
 * Be consistent about your usage of X vs Y, you can only send one or the other.
 * @author M. Francis
 */
public class PixyCam2Wire extends SubsystemBase {

  // Ports
  final private AnalogInput analog;
  final private DigitalInput digital;

  final private int FINAL_READING;

  // Debug mode
  private final boolean DEBUG = true;

  /**
   * Creates a PixyCamAnalog object
   * 
   * @param analogPort  The analog port used for the PixyCam
   * @param digitalPort The digital port used for the PixyCam
   */
  public PixyCam2Wire(int analogPort, int digitalPort) {
    this.analog = new AnalogInput(analogPort);
    this.digital = new DigitalInput(digitalPort);
    FINAL_READING = getLargestTargetX();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Pixy sees target", pixySeesTarget());

    if (DEBUG) {
      SmartDashboard.putNumber("Pixy Voltage", getPixyVoltage());
      SmartDashboard.putNumber("Pixy Target Angle", getPixyTargetAngleX());
      SmartDashboard.putNumber("Pixy Target X-Position", getLargestTargetX());

      var pathOptional = getPath();
      if (pathOptional.isEmpty()) {
        SmartDashboard.putString("Pixy Path", "No path");
      } else {
        var path = pathOptional.get();
        switch(path){
          case RED_A:
            SmartDashboard.putString("Pixy Path", "Red A");
            break;
          case RED_B:
            SmartDashboard.putString("Pixy Path", "Red B");
            break;
          case BLUE_A:
            SmartDashboard.putString("Pixy Path", "Blue A");
            break;
          case BLUE_B:
            SmartDashboard.putString("Pixy Path", "Blue B");
            break;
        }
      }
    }
  }

  /**
   * @return The voltage from the analog port as a raw value.
   * This is the largest target value.
   */
  private double getPixyVoltage() {
    return analog.getVoltage();
  }

  /**
   * @return Returns true if the PixyCam sees a target.
   */
  public boolean pixySeesTarget() {
    return digital.get();
  }

  /**
   * @return The angle in degrees of the largest target from the center of the
   * PixyCam. Use this method if the PixyCam is sending x information over the 
   * analog port. This will range from -30 to 30 as the horizontal field of
   * view of a Pixy2 is 60 degrees. Returns 0 if there is no target.
   */
  public double getPixyTargetAngleX() {
    if(!pixySeesTarget()) {
      return 0;
    }
    return ((getPixyVoltage() / 3.3) * 60) - 30;
  }

  /**
   * @return The position in pixels of the largest target from the left side of
   * the PixyCam. Use this method if the PixyCam is sending x information over
   * the analog port. This will range from 0 to 315. Returns -1 if no target
   * was found.
   */
  public int getLargestTargetX() {
    if(!pixySeesTarget()) {
      return -1;
    }
    int x = (int)((getPixyVoltage() / 3.3) * 315);

    return x;
  }

  public enum GalacticSearchPath {
    RED_A,
    RED_B,
    BLUE_A,
    BLUE_B
  }

  public Optional<GalacticSearchPath> getPath() {
    int x = getLargestTargetX();

    if (x >= 220 && x < 235) {
      //Red A path
      return Optional.of(GalacticSearchPath.RED_A);
    } else if (x >= 0 && x < 100) {
      //Red B path
      return Optional.of(GalacticSearchPath.RED_B);
    } else if (x >= 200 && x < 220) {
      //Blue A path
      return Optional.of(GalacticSearchPath.BLUE_A);
    } else if (x >= 150 && x <= 200) {
      //Blue B path
      return Optional.of(GalacticSearchPath.BLUE_B);
    }
    return Optional.empty();
  }

  public void printPath(){
    int x = FINAL_READING;
    
    if (x >= 220 && x < 235) {
      //Red A path
      System.out.println("Red A");
    } else if (x >= 0 && x < 100) {
      //Red B path
      System.out.println("Red B");
    } else if (x >= 200 && x < 220) {
      //Blue A path
      System.out.println("Blue A");
    } else if (x >= 150 && x <= 200) {
      //Blue B path
      System.out.println("Blue B");
    }
  }

}
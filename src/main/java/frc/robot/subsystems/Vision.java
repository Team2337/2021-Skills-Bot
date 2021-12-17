package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

  /**
   * The vision code for the limelight and camera
   * @author EngiNERDs
   * @category VISION
   */
public class Vision extends SubsystemBase {
  public AnalogInput pixyLeftAnalog;
  public AnalogInput pixyRightAnalog;
  public DigitalInput pixyLeftDigital;
  public DigitalInput pixyRightDigital;
  
  private boolean rotateLimelight = false;
  private boolean pixyDebug = false;

  public Vision() {
   /* pixyRightAnalog = new AnalogInput(4);
    pixyRightDigital = new DigitalInput(5);
    pixyLeftAnalog = new AnalogInput(5);
    pixyLeftDigital = new DigitalInput(6); */
  }

  /// LIMELIGHT  ////////////////////

  /**
   * Sets the LED mode to on, off, or blink
   * @param mode - the mode of the LEDs
   * Example: 
   * 0: Sets the mode to what is in the current pipeline
   * 1: Turns off the LEDs
   * 2: Blink mode on LEDs
   * 3: Turns on the LEDs
   */
   public void setLEDMode(int mode) {
     NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
   }

   /**
    * Gets the Limelight mode number from the NetworkTable
    * @return - returns the mode number from the NetworkTable 
    */
   public int getLEDMode() {
     return (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getValue().getDouble();
   }

   /**
    * Sets the pipeline of the limelight
    * @param pipeline - sets the desired pipeline number between 0-9
    * 0 - CloseVision (0-15 ft away from the tower)
    * 1 - MediumVision (between 15-21 ft)
    * 2 - FarVision (21-26 ft)
    * 9 - Drivecam
    */
   public void switchPipeLine(int pipeline) {
    double currentPipeline = getPipeline();
    if(currentPipeline != pipeline){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
   }

   /**
    * Gets the current pipeline on the limelight
    * @return - Double value limelight pipeline (0 -> 9)
    */
   public double getPipeline() {
     return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
   }

   /**
    * Checks if Limelight has a target.
    * @return - returns whether the limelight has any valid targets (0 or 1)
    */
   public boolean ifLimelightHasTarget() {
     return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1);
   }
   
   /**
    * Checks Limelight target yaw.
    * @return - returns Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    */
    public double getLimelightYawDegrees() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
       
   /**
    * Checks Limelight target pitch.
    * @return - returns Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    * ta	Target Area (0% of image to 100% of image)
    */
    public double getLimelightPitchDegrees() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

   /**
    * Checks Limelight target area.
    * @return - returns Target Area (0% of image to 100% of image)
    */
    public double getLimelightArea() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }

   /**
    * Set Boolean value to inform us of we are rotating via limelight
    * @param rotateLimelight - Boolean value (limelight mode: true | not limelight mode: false)
    */
   public void setRotateLimelight(boolean rotateLimelight) {
    this.rotateLimelight = rotateLimelight;
   }

  /**
   * Lets us know if we are in the limelight mode, we are rotating using the limelight
   * @return - Boolean value, rotating via limelight
   */
   public boolean getRotateLimelight() {
    return rotateLimelight;
  }


  /// OPENSIGHT  ////////////////////

    /**
     * This will get the X coordinte from Opensight
     * @return - returns the x-coordinate value, which will show how far we need to rotate
     */
    public double getOpenSightXCoordinateValue() {
      return NetworkTableInstance.getDefault().getTable("PutCoordinate").getEntry("coord-x").getDouble(0);

  }
    /**
     * This will get the Y coordinate from Opensight
     * @return - returns the y-coordinate value, which will show how far the robot needs to drive
     */
    public double getOpenSightYCoordinateValue() {
      return NetworkTableInstance.getDefault().getTable("PutCoordinate").getEntry("coord-y").getDouble(0);
  }
  /**
   * Receives the NetworkTable values from Opensight
   * @return - returns the opensight networktable which will return a true/false value
   */
  public boolean getOpenSightNTValue() {
    return NetworkTableInstance.getDefault().getTable("PutNT").getEntry("succ").getBoolean(false);
  }
}
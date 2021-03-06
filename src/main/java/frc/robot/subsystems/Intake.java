package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Subsystem for the intake
 * @author Michael F.
 */
public class Intake extends SubsystemBase {

  private final boolean DEBUG = true;

  private TalonFX intakeMotor;
  private StatorCurrentLimitConfiguration intakeCurrentLimitConfig;

  private double setSpeed;

  public Intake(){
    //Initialize variables
    setSpeed = 0;

    //Initialize motor
    intakeMotor = new TalonFX(Constants.INTAKE);

    //Set settings on motor
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);

    //Configure a current limit
    intakeCurrentLimitConfig = new StatorCurrentLimitConfiguration();
    intakeCurrentLimitConfig.currentLimit = 50;
    intakeCurrentLimitConfig.enable = true;
    intakeCurrentLimitConfig.triggerThresholdCurrent = 40;
    intakeCurrentLimitConfig.triggerThresholdTime = 3;
    //Push the current limit to the motor
    intakeMotor.configStatorCurrentLimit(intakeCurrentLimitConfig, 0);

    //Configure motor ramp rate
    intakeMotor.configClosedloopRamp(0.5);
  }

  @Override
  public void periodic(){
    //Debug stuff
    if(DEBUG){
      SmartDashboard.putNumber("Intake Set Speed", getIntakeSetSpeed());
      SmartDashboard.putNumber("Intake Actual Speed", getIntakeActualSpeed());
      SmartDashboard.putNumber("Intake Motor Temp", getIntakeTemperature());
    }
  }

  /**
   * Sets the speed of the intake
   * @param speed The speed to set the motor as a percentage value -1 through 1.
   */
  public void setIntakeSpeed(double speed){
    //Set tracking variable
    setSpeed = speed;

    //Set intake speed
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops the motor
   */
  public void stopIntake(){
    //Set tracking variable
    setSpeed = 0;

    //Stop motor by setting its speed to 0
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return The set speed of the motor
   */
  public double getIntakeSetSpeed(){
    return setSpeed;
  }

  /**
   * @return The current speed of the motor
   */
  public double getIntakeActualSpeed(){
    return intakeMotor.getMotorOutputPercent();
  }

  /**
   * Gets the temperature of the intake motor
   * @return The temperature in Celsius of the intake motor
   */
  public double getIntakeTemperature(){
    return intakeMotor.getTemperature();
  }
}

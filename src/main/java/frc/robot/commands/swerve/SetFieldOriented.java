/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Sets the field orientation mode 
 * @see SwerveDrivetrain
 * @author Bryce G.
 * @category SWERVE
 */
public class SetFieldOriented extends InstantCommand {

  private boolean isFieldOriented;
  private SwerveDrivetrain subsystem;
 
  /**
   * Sets the field orientation mode
   * @see SwerveDrivetrain
   * @param isFieldOriented - boolean value to set the field orientation mode
   */
  public SetFieldOriented(SwerveDrivetrain subsystem, boolean isFieldOriented) {
    this.isFieldOriented = isFieldOriented;
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      subsystem.setFieldOriented(this.isFieldOriented);
      SmartDashboard.putBoolean("command field oriented", this.isFieldOriented);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}

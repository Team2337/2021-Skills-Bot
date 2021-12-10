// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.commands.commandgroups.CGGalaticSearchBlueA;
import frc.robot.commands.commandgroups.CGGalaticSearchBlueB;
import frc.robot.commands.commandgroups.CGGalaticSearchRedA;
import frc.robot.commands.commandgroups.CGGalaticSearchRedB;
import frc.robot.commands.auto.autonav.BarrelRacing;
import frc.robot.commands.auto.autonav.BarrelRacing2;
import frc.robot.commands.auto.autonav.Bounce;
import frc.robot.commands.auto.autonav.Slalom;
import frc.robot.commands.auto.autonav.Slalom2;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController driverController = new XboxController(0);
  // private final XboxController operatorController = new XboxController(1);

  /* --- Subsystems --- */
  public PixyCam2Wire pixy = new PixyCam2Wire(Constants.PIXY_ANALOG, Constants.PIXY_DIGITAL);
  private Pigeon pigeon = new Pigeon();
  private SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(pigeon);
  private Intake intake = new Intake();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Driver Left Bumper is used for field-oriented drive - held for true, released for false
    swerveDrivetrain.setDefaultCommand(new SwerveDriveCommand(
      () -> -Utilities.modifyAxis(driverController.getY(GenericHID.Hand.kLeft)) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -Utilities.modifyAxis(driverController.getX(GenericHID.Hand.kLeft)) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -Utilities.modifyAxis(driverController.getX(GenericHID.Hand.kRight)) * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      () -> !driverController.getBumper(GenericHID.Hand.kLeft),
      swerveDrivetrain
    ));

    // Configure the button bindings
    configureButtonBindings();

    // Resets the pigeon to 0
    pigeon.resetPidgey();

    autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    try { autonChooser.addOption("Barrel Racing", new BarrelRacing(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Barrel Racing 2 (Centr)", new BarrelRacing2(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Bounce", new Bounce(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Slalom", new Slalom(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Slalom 2 (Centr)", new Slalom2(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }

    autonChooser.addOption("Galactic Search", new GalacticSearch(pixy, swerveDrivetrain));
    try { autonChooser.addOption("CGGalatic Search Red A", new CGGalaticSearchRedA(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("CGGalatic Search Red B", new CGGalaticSearchRedB(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("CGGalatic Search Blue A", new CGGalaticSearchBlueA(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("CGGalatic Search Blue B", new CGGalaticSearchBlueB(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton redB = new JoystickButton(driverController, XboxController.Button.kB.value);

    final JoystickButton bumperRight = new JoystickButton(driverController, XboxController.Button.kBumperRight.value);
    final JoystickButton bumperLeft = new JoystickButton(driverController, XboxController.Button.kBumperLeft.value);

    //Drive motor controls
    redB.whenPressed(() -> swerveDrivetrain.resetOdometry());

    // Intake controls
    bumperRight.whenPressed(new SetIntakeSpeed(intake, 0.75));
    bumperRight.whenReleased(new SetIntakeSpeed(intake, 0));

    bumperLeft.whenPressed(new SetIntakeSpeed(intake, -0.25));
    bumperLeft.whenReleased(new SetIntakeSpeed(intake, 0));

    SmartDashboard.putData("AutonChooser", autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public void resetDrivetrainOdometry() {
    swerveDrivetrain.resetOdometry();
  }

}

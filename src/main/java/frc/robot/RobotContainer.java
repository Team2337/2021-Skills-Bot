// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.LPathTrajectory;
import frc.robot.commands.auto.MotionMagicCommand;
import frc.robot.commands.auto.autonav.BarrelRacing;
import frc.robot.commands.auto.autonav.BarrelRacing2;
import frc.robot.commands.auto.autonav.Bounce;
import frc.robot.commands.auto.autonav.Slalom;
import frc.robot.commands.auto.autonav.Slalom2;
import frc.robot.commands.swerve.SwerveDriveCommand;
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
  private final XboxController operatorController = new XboxController(1);

  /* --- Subsystems --- */
  private Pigeon pigeon = new Pigeon();
  private SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(pigeon);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrivetrain.setDefaultCommand(new SwerveDriveCommand(swerveDrivetrain, driverController, operatorController, pigeon));

    // Configure the button bindings
    configureButtonBindings();

    // Resets the pigeon to 0
    pigeon.resetPidgey();

    resetDrivetrain();

    autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    try { autonChooser.addOption("Barrel Racing", new BarrelRacing(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Barrel Racing 2 (Centr)", new BarrelRacing2(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Bounce", new Bounce(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Slalom", new Slalom(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Slalom 2 (Centr)", new Slalom2(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }


    autonChooser.addOption("LPathCommand", new LPathTrajectory(swerveDrivetrain));
    autonChooser.addOption("Motion Magic (10ft)", new MotionMagicCommand(new Translation2d(10, 5), swerveDrivetrain));
    autonChooser.addOption("Motion Magic (L-10ft)", new SequentialCommandGroup(
      new MotionMagicCommand(new Translation2d(5, 2.5), swerveDrivetrain),
      new MotionMagicCommand(new Translation2d(10, 5), swerveDrivetrain)
    ));
    autonChooser.addOption("Motion Magic (L-10ft 2)",
        new SequentialCommandGroup(new MotionMagicCommand(new Translation2d(5, 0), swerveDrivetrain),
        new MotionMagicCommand(new Translation2d(10, 5), swerveDrivetrain)));
    autonChooser.addOption("Motion Magic (Tuning 1)", new SequentialCommandGroup(
        new MotionMagicCommand(new Translation2d(5, 0), swerveDrivetrain)
    ));
    autonChooser.addOption("Motion Magic (Tuning 2)", new SequentialCommandGroup(
        new MotionMagicCommand(new Translation2d(10, 0), swerveDrivetrain),
        new MotionMagicCommand(new Translation2d(5, 0), swerveDrivetrain),
        new MotionMagicCommand(new Translation2d(10, 0), swerveDrivetrain)
    ));
    autonChooser.addOption("Motion Magic (Tuning 3)", new SequentialCommandGroup(
        new MotionMagicCommand(new Translation2d(-5, 0), swerveDrivetrain)
    ));

  }

  public void resetDrivetrain() {
    swerveDrivetrain.resetOdometry();
    swerveDrivetrain.resetDriveMotors();
    swerveDrivetrain.resetAngleMotors();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Left Bumper is used for field-oriented drive - held for true, released
    // for false

    final JoystickButton greenA = new JoystickButton(driverController, XboxController.Button.kA.value);
    final JoystickButton redB = new JoystickButton(driverController, XboxController.Button.kB.value);
    final JoystickButton blueX = new JoystickButton(driverController, XboxController.Button.kX.value);
    final JoystickButton yellowY = new JoystickButton(driverController, XboxController.Button.kY.value);
    final JoystickButton rightStarButton = new JoystickButton(driverController, XboxController.Button.kStart.value);

    final JoystickButton operatorRedB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    final JoystickButton operatorGreenA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    final JoystickButton operatorBlueX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    final JoystickButton operatorYellowY = new JoystickButton(operatorController, XboxController.Button.kY.value);




    final JoystickButton bumperRight = new JoystickButton(driverController, XboxController.Button.kBumperRight.value);
    final JoystickButton bumperLeft = new JoystickButton(driverController, XboxController.Button.kBumperLeft.value);

    //Drive motor controls
    greenA.whenPressed(() -> swerveDrivetrain.resetDriveMotors());
    // greenA.whenPressed(new InstantCommand(() -> swerveDrivetrain.resetDriveEncoders())));
    redB.whenPressed(() -> swerveDrivetrain.resetOdometry());

    //blueX.whenPressed(new RotateToDegree(90, swerveDrivetrain, pigeon).withTimeout(1.3));
    //yellowY.whenPressed(new RotateToDegree(1, swerveDrivetrain, pigeon).withTimeout(1.3));

    blueX.whenPressed(() -> swerveDrivetrain.setFieldOrientedOffset());
    operatorYellowY.whenPressed(() -> swerveDrivetrain.setFutureFieldOrientedOffset(0));
    operatorRedB.whenPressed(() -> swerveDrivetrain.setFutureFieldOrientedOffset(-90));
    operatorGreenA.whenPressed(() -> swerveDrivetrain.setFutureFieldOrientedOffset(-180));
    operatorBlueX.whenPressed(() -> swerveDrivetrain.setFutureFieldOrientedOffset(-270));

    SmartDashboard.putData("AutonChooser", autonChooser);
    // SmartDashboard.putData("Reset Drive Encoder", new InstantCommand(() -> swerveDrivetrain.resetDriveEncoders())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

}

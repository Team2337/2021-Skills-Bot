// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.commands.auto.LPathTrajectory;
import frc.robot.commands.auto.MotionMagicCommand;
import frc.robot.commands.auto.calibration.StraightLineTest10Ft;
import frc.robot.commands.auto.calibration.StraightLineTest10Ft0;
import frc.robot.commands.auto.calibration.StraightLineTest10Ft1;
import frc.robot.commands.commandgroups.CGGalaticSearchBlueA;
import frc.robot.commands.commandgroups.CGGalaticSearchBlueB;
import frc.robot.commands.commandgroups.CGGalaticSearchRedA;
import frc.robot.commands.commandgroups.CGGalaticSearchRedB;
import frc.robot.commands.heading.Heading;
import frc.robot.commands.heading.HeadingSupplier;
import frc.robot.commands.heading.PathFinderCommand;
import frc.robot.commands.auto.autonav.BarrelRacing;
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
  private final XboxController operatorController = new XboxController(1);

  /* --- Subsystems --- */
  private PixyCam2Wire pixy = new PixyCam2Wire(Constants.PIXY_ANALOG, Constants.PIXY_DIGITAL);
  private Pigeon pigeon = new Pigeon();
  private SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(pigeon);
  private Intake intake = new Intake();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrivetrain.setDefaultCommand(new SwerveDriveCommand(swerveDrivetrain, driverController, operatorController));

    // Configure the button bindings
    configureButtonBindings();

    // Resets the pigeon to 0
    pigeon.resetPidgey();

    resetDrivetrain();

    autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    try { autonChooser.addOption("Barrel Racing", new BarrelRacing(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    // try { autonChooser.addOption("Barrel Racing (PathFinder)", new BarrelRacing(swerveDrivetrain).alongWith(new PathFinderCommand<Double>(
    //   List.of(
    //     new Translation2d(Units.feetToMeters(4.0), Units.feetToMeters(9.0)),
    //     new Translation2d(Units.feetToMeters(13.867001027252401), Units.feetToMeters(6.538763671520938)),
    //     new Translation2d(Units.feetToMeters(11.792253308357), Units.feetToMeters(2.2560275545350184)),
    //     new Translation2d(Units.feetToMeters(9.489092996555682), Units.feetToMeters(4.84470360746873)),
    //     new Translation2d(Units.feetToMeters(11.068946764154934), Units.feetToMeters(7.395310894918123)),
    //     new Translation2d(Units.feetToMeters(18.5), Units.feetToMeters(8.3)),
    //     new Translation2d(Units.feetToMeters(21.8), Units.feetToMeters(11.2)),
    //     new Translation2d(Units.feetToMeters(18.987249984893346), Units.feetToMeters(14.133482385642637)),
    //     new Translation2d(Units.feetToMeters(16.5), Units.feetToMeters(10.5)),
    //     new Translation2d(Units.feetToMeters(20.25), Units.feetToMeters(5.5)),
    //     new Translation2d(Units.feetToMeters(25.21149314157955), Units.feetToMeters(3.0)),
    //     new Translation2d(Units.feetToMeters(27.800169194513263), Units.feetToMeters(6.1)),
    //     new Translation2d(Units.feetToMeters(23.15577980542631), Units.feetToMeters(9.0)),
    //     new Translation2d(Units.feetToMeters(0.23838298386609455), Units.feetToMeters(8.5)),
    //     new Translation2d(Units.feetToMeters(1.0), Units.feetToMeters(8.5))
    //   ),
    //   () -> swerveDrivetrain.getPose().getTranslation(),
    //   swerveDrivetrain::getEncoderDistanceInches
    // ))); } catch (IOException e) { e.printStackTrace(); }
    // try { autonChooser.addOption("Barrel Racing (Heading)", new BarrelRacing(new HeadingSupplier(
    //   List.of(
    //     new Heading(0.0, new Rotation2d())
    //   ),
    //   swerveDrivetrain::getEncoderDistanceInches
    // ), swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }

    try { autonChooser.addOption("Bounce", new Bounce(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }

    try { autonChooser.addOption("Slalom", new Slalom(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }

    Command slalom2;
    List<Pose2d> slalomPoses = List.of();
    try {
      Slalom2 localSlalom = new Slalom2(swerveDrivetrain);
      slalom2 = localSlalom;
      slalomPoses = localSlalom.getPoses();
    } catch (IOException e) {
      slalom2 = new WaitCommand(15);
    }
    autonChooser.addOption("Slalom (PathFinder)", slalom2.alongWith(new PathFinderCommand<Double>(
      slalomPoses,
      () -> swerveDrivetrain.getPose().getTranslation(),
      swerveDrivetrain::getEncoderDistanceInches
    )));
    
    try { autonChooser.addOption("Slalom (Heading)", new Slalom2(new HeadingSupplier(
      List.of(
        new Heading(0.0, Rotation2d.fromDegrees(0)),
        new Heading(17.835534464637018, Rotation2d.fromDegrees(-90)), // D4ish
        new Heading(109.99394200506657, Rotation2d.fromDegrees(-135)), // D8ish top
        new Heading(255.0709269707177, Rotation2d.fromDegrees(-75)), // D10 bottom
        new Heading(335.55679344725075, Rotation2d.fromDegrees(0)), // End of Circle
        new Heading(403.0902976339161, Rotation2d.fromDegrees(90)), // D10 top
        new Heading(467.71149417769857, Rotation2d.fromDegrees(135)), // D8ish bottom
        new Heading(551.7240125636354, Rotation2d.fromDegrees(90)), // D6 bottom
        new Heading(624.9715951851748, Rotation2d.fromDegrees(45)) // D4ish
      ),
      swerveDrivetrain::getEncoderDistanceInches
    ), swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }

    try { autonChooser.addOption("StraightLineTest10Ft", new StraightLineTest10Ft(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("StraightLineTest10Ft0", new StraightLineTest10Ft0(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("StraightLineTest10Ft1", new StraightLineTest10Ft1(swerveDrivetrain));} catch (IOException e) { e.printStackTrace(); }

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

    autonChooser.addOption("Galactic Search", new GalacticSearch(pixy, swerveDrivetrain));
    // try { autonChooser.addOption("Galatic Search Red A", new GalacticSearchRedA(swerveDrivetrain).beforeStarting(new SetIntakeSpeed(intake, 1).withTimeout(2); } catch (IOException e) { e.printStackTrace(); }
    // try { autonChooser.addOption("Galatic Search Red B", new GalacticSearchRedB(swerveDrivetrain).beforeStarting(() -> intake.setIntakeSpeed(1), intake));} catch (IOException e) { e.printStackTrace(); }
    // try { autonChooser.addOption("Galatic Search Blue A", new GalacticSearchBlueA(swerveDrivetrain).beforeStarting(() -> intake.setIntakeSpeed(1), intake));} catch (IOException e) { e.printStackTrace(); }
    // try { autonChooser.addOption("Galatic Search Blue B", new GalacticSearchBlueB(swerveDrivetrain).beforeStarting(() -> intake.setIntakeSpeed(1), intake));} catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("CGGalatic Search Red A", new CGGalaticSearchRedA(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("CGGalatic Search Red B", new CGGalaticSearchRedB(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("CGGalatic Search Blue A", new CGGalaticSearchBlueA(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("CGGalatic Search Blue B", new CGGalaticSearchBlueB(swerveDrivetrain, intake)); } catch (IOException e) { e.printStackTrace(); }
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

    final JoystickButton bumperRight = new JoystickButton(driverController, XboxController.Button.kBumperRight.value);

    //Drive motor controls
    greenA.whenPressed(() -> swerveDrivetrain.resetDriveMotors());
    // greenA.whenPressed(new InstantCommand(() -> swerveDrivetrain.resetDriveEncoders())));
    redB.whenPressed(() -> swerveDrivetrain.resetOdometry());

    // Intake controls
    bumperRight.whenPressed(new SetIntakeSpeed(intake, 0.75));
    bumperRight.whenReleased(new SetIntakeSpeed(intake, 0));

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

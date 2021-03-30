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
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.commands.auto.LPathTrajectory;
import frc.robot.commands.auto.MotionMagicCommand;
import frc.robot.commands.auto.calibration.StraightLineTest10Ft;
import frc.robot.commands.auto.calibration.StraightLineTest10Ft0;
import frc.robot.commands.auto.calibration.StraightLineTest10Ft1;
import frc.robot.commands.auto.galacticsearch.GalacticSearchBlueA;
import frc.robot.commands.auto.galacticsearch.GalacticSearchBlueB;
import frc.robot.commands.auto.galacticsearch.GalacticSearchRedA;
import frc.robot.commands.auto.galacticsearch.GalacticSearchRedB;
import frc.robot.commands.auto.commandGroups.CircleTest;
import frc.robot.commands.auto.autonav.BarrelRacing;
import frc.robot.commands.auto.autonav.Bounce;
import frc.robot.commands.auto.autonav.Slalom;
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

  private final XboxController controller = new XboxController(0);

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
    swerveDrivetrain.setDefaultCommand(new SwerveDriveCommand(swerveDrivetrain, controller));

    // Configure the button bindings
    configureButtonBindings();

    // Resets the pigeon to 0
    pigeon.resetPidgey();

    resetDrivetrain();

    autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    try { autonChooser.addOption("Barrel Racing", new BarrelRacing(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Bounce", new Bounce(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Slalom", new Slalom(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("StraightLineTest10Ft", new StraightLineTest10Ft(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("StraightLineTest10Ft0", new StraightLineTest10Ft0(swerveDrivetrain)); } catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("StraightLineTest10Ft1", new StraightLineTest10Ft1(swerveDrivetrain));} catch (IOException e) { e.printStackTrace(); }

    autonChooser.addOption("CircleTest", new CircleTest(swerveDrivetrain));
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
    try { autonChooser.addOption("Galatic Search Red A", new GalacticSearchRedA(swerveDrivetrain));} catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Galatic Search Red B", new GalacticSearchRedB(swerveDrivetrain));} catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Galatic Search Blue A", new GalacticSearchBlueA(swerveDrivetrain));} catch (IOException e) { e.printStackTrace(); }
    try { autonChooser.addOption("Galatic Search Blue B", new GalacticSearchBlueB(swerveDrivetrain));} catch (IOException e) { e.printStackTrace(); }
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

    final JoystickButton greenA = new JoystickButton(controller, XboxController.Button.kA.value);
    final JoystickButton redB = new JoystickButton(controller, XboxController.Button.kB.value);
    final JoystickButton blueX = new JoystickButton(controller, XboxController.Button.kX.value);

    final JoystickButton bumperRight = new JoystickButton(controller, XboxController.Button.kBumperRight.value);
    final JoystickButton yellowY = new JoystickButton(controller, XboxController.Button.kY.value);

    //Drive motor controls
    greenA.whenPressed(() -> swerveDrivetrain.resetDriveMotors());
    // greenA.whenPressed(new InstantCommand(() -> swerveDrivetrain.resetDriveEncoders())));
    redB.whenPressed(() -> swerveDrivetrain.resetOdometry());
    //blueX.whileHeld(() -> swerveDrivetrain.setAngleMotorsTeleop(45));
    //yellowY.whileHeld(() -> swerveDrivetrain.setAngleMotorsTeleop(-45));

    // Intake controls
    bumperRight.whenPressed(new SetIntakeSpeed(intake, 0.5));
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

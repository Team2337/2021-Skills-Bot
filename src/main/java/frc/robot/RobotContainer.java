// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.autonav.BarrelRacingCommand;
import frc.robot.commands.auto.autonav.BounceCommand;
import frc.robot.commands.auto.autonav.SlalomCommand;
import frc.robot.commands.auto.GalacticSearchCommand;
import frc.robot.commands.swerve.SetTurnMotorTicks;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.subsystems.PixyCam;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

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
  private PixyCam pixy = new PixyCam(0);
  private Pigeon pigeon = new Pigeon();
  private SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(pigeon);

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

    autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    try {
      autonChooser.addOption("Barrel Racing", new BarrelRacingCommand(swerveDrivetrain));
    } catch (IOException e) {
      e.printStackTrace();
    }
    try {
      autonChooser.addOption("Bounce", new BounceCommand(swerveDrivetrain));
    } catch (IOException e) {
      e.printStackTrace();
    }
    try {
      autonChooser.addOption("Slalom", new SlalomCommand(swerveDrivetrain));
    } catch (IOException e) {
      e.printStackTrace();
    }
    autonChooser.addOption("Galactic Search", new GalacticSearchCommand(pixy, swerveDrivetrain));
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

    final JoystickButton greenA = new JoystickButton(controller, 1);
    final JoystickButton redB = new JoystickButton(controller, 2);

    greenA.whenPressed(new SetTurnMotorTicks(swerveDrivetrain));
    redB.whenPressed(new SetTurnMotorTicks(swerveDrivetrain).withTimeout(1));
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

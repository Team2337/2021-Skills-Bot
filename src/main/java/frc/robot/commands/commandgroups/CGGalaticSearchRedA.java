package frc.robot.commands.commandgroups;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.galacticsearch.GalacticSearchRedA;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrivetrain;

public class CGGalaticSearchRedA extends SequentialCommandGroup {

  public CGGalaticSearchRedA(SwerveDrivetrain drivetrain, Intake intake) throws IOException {
        addCommands(
          new SetIntakeSpeed(intake, 0.75).withTimeout(5),
          new GalacticSearchRedA(drivetrain)
        );
    }

}

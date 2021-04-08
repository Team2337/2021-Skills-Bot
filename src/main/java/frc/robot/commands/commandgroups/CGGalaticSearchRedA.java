package frc.robot.commands.commandgroups;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.galacticsearch.GalacticSearchRedA;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrivetrain;

public class CGGalaticSearchRedA extends ParallelCommandGroup {

  public CGGalaticSearchRedA(SwerveDrivetrain drivetrain, Intake intake) throws IOException {
        addCommands(
          new GalacticSearchRedA(drivetrain),
          new WaitCommand(0.2).andThen(new SetIntakeSpeed(intake, 0.75).withTimeout(2.6)))
        ;
    }

}

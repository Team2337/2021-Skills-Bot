package frc.robot.commands.commandgroups;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.galacticsearch.GalacticSearchRedB;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrivetrain;

public class CGGalaticSearchRedB extends ParallelCommandGroup {

  public CGGalaticSearchRedB(SwerveDrivetrain drivetrain, Intake intake) throws IOException {
        addCommands(
          new SetIntakeSpeed(intake, 0.75).withTimeout(3.0),
          new GalacticSearchRedB(drivetrain))
        ;
    }

}

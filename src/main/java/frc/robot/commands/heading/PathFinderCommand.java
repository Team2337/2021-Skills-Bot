package frc.robot.commands.heading;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathFinderCommand<T> extends CommandBase {

  private PathFinder<T> pathFinder;
  private Supplier<Translation2d> translationMetersSupplier;
  private Supplier<T> valueSupplier;

  public PathFinderCommand(List<Pose2d> poses, Supplier<Translation2d> translationMetersSupplier, Supplier<T> valueSupplier) {
    pathFinder = new PathFinder<T>(poses.stream().map(p -> p.getTranslation()).collect(Collectors.toList()));
    this.translationMetersSupplier = translationMetersSupplier;
    this.valueSupplier = valueSupplier;
  }

  @Override
  public void execute() {
    Translation2d currentTranslationMeters = translationMetersSupplier.get();
    T currentValue = valueSupplier.get();
    pathFinder.addSample(currentTranslationMeters, currentValue);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.printf("Waypoints: %s%n", pathFinder.getResults().toString());
  }

}

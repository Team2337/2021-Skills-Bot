package frc.robot.commands.heading;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class PathFinder<T> {

  // Key is a Translation2d, value is an optional distance for the waypoint
  private HashMap<Translation2d, Optional<T>> results = new HashMap<Translation2d, Optional<T>>();

  // public PathFinder(Iterable<Translation2d> waypoints, Supplier<Translation2d> translationSupplier, Supplier<Comparable> valueSupplier) {
  public PathFinder(Iterable<Translation2d> translationsMeters) {
    for (Translation2d translation : translationsMeters) {
      this.results.put(translation, Optional.empty());
    }
  }

  public void addSample(Translation2d currentTranslationMeters, T currentValue) {
     // Find closest Translation2d for the given Translation2d
    Optional<Translation2d> translation = Optional.empty();
    for (Translation2d t : results.keySet()) {
      // If any given point is too far away from the current translation, discard it
      if (Math.abs(currentTranslationMeters.getDistance(t)) > Units.inchesToMeters(6)) {
        continue;
      }

      if (translation.isEmpty()) {
        translation = Optional.of(t);
      } else {
        Translation2d previousTranslation = translation.get();
        // If the translation we're evaluating is closer than the previous key translation to the
        // current translation - update the key translation to be the closer one
        if (Math.abs(currentTranslationMeters.getDistance(t)) < Math.abs(currentTranslationMeters.getDistance(previousTranslation))) {
          translation = Optional.of(t);
        }
      }
    }

    // Current sample is not close enough to any given point - discard
    if (translation.isEmpty()) {
      return;
    }
    results.put(translation.get(), Optional.of(currentValue));
  }

  public HashMap<Translation2d, Optional<T>> getResults() {
    return results;
  }

}

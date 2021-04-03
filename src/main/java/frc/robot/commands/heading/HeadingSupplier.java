package frc.robot.commands.heading;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class HeadingSupplier implements Supplier<Rotation2d> {

  private List<Heading> headings;
  private Supplier<Double> valueSupplier;

  public HeadingSupplier(List<Heading> headings, Supplier<Double> valueSupplier) {
    this.headings = headings;
    this.valueSupplier = valueSupplier;
  }

  public Rotation2d get() {
    Double value = valueSupplier.get();
    return sample(value).rotation;
  }

  private Heading sample(Double value) {
    if (value <= headings.get(0).value) {
      return headings.get(0);
    }
    Heading finalHeading = headings.get(headings.size() - 1);
    if (value >= finalHeading.value) {
      return finalHeading;
    }

    // To get the element that we want, we will use a binary search algorithm
    // instead of iterating over a for-loop. A binary search is O(std::log(n))
    // whereas searching using a loop is O(n).

    // This starts at 1 because we use the previous state later on for
    // interpolation.
    int low = 1;
    int high = headings.size() - 1;

    while (low != high) {
      int mid = (low + high) / 2;
      if (headings.get(mid).value < value) {
        // This index and everything under it are less than the requested
        // timestamp. Therefore, we can discard them.
        low = mid + 1;
      } else {
        // t is at least as large as the element at this index. This means that
        // anything after it cannot be what we are looking for.
        high = mid;
      }
    }

    // High and Low should be the same.

    // The sample's timestamp is now greater than or equal to the requested
    // timestamp. If it is greater, we need to interpolate between the
    // previous state and the current state to get the exact state that we
    // want.
    final Heading sample = headings.get(low);
    final Heading prevSample = headings.get(low - 1);

    // If the difference in states is negligible, then we are spot on!
    if (Math.abs(sample.value - prevSample.value) < 1E-9) {
      return sample;
    }
    // Interpolate between the two states for the state that we want.
    return prevSample.interpolate(sample, (value - prevSample.value) / (sample.value - prevSample.value));
  }

}

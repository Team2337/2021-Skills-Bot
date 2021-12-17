package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Utilities class to house basic methods that can be used across subsystems
 * 
 * @author Bryce G.
 */
public class Utilities {
    /**
     * Checks to see if the absolute value of the input is less than the deadband
     * @param input - Value in which the deadband will be applied (0 < input < 1)
     * @param deadband - Deadband to set on the input (double)
     * @return - input double value adjusted for the deadband
     */
    public static double deadband(double input, double deadband) {
      if (Math.abs(input) < deadband) return 0;
		  return Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
    }

    /**
     * Squares the input value
     * @param value - double value wanting to be squared
     * @return - squared input double value
     */
    public static double squareValues(double value) {
      return Math.copySign(Math.pow(value, 2), value);
    }

    public static double modifyAxis(double value) {
      // Deadband
      value = deadband(value, 0.05);
      // Square the axis
      return Math.copySign(value * value, value);
    }

    /**
     * Calculates derivative based on the current and previous sensor inputs
     * @param error - double value reading the current sensor error (current - target)
     * @param lastError - double value reading the previous sensor error (current - target)
     * @param dt - Change in time from when the previous sensor error was gotten to the time the current was gotten
     * @return - returns derivative double value to add to the speed of the motor
     */
    public static double calculateDerivative(double error, double lastError, double dt) {
      if (Double.isFinite(lastError)) {
        return (error - lastError) / dt;
      } else {
        return 0;
      }
    }

  /**
   * Determines whether or not the given value is within a certain amount of a target
   *
   * @param target The desired value
   * @param current The current value
   * @param tolerance A range that the given value can be within the target value before returning true
   * @return Whether or not the current value is within a tolerance of the target
   */
  public static boolean withinTolerance(double target, double current, double tolerance) {
    return Math.abs(target - current) <= tolerance;
  }

  /**
    * Algorithm from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html Estimates
    * range to a target using the target's elevation. This method can produce more stable results
    * than SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
    * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor), and
    * for there to exist a height differential between goal and camera. The larger this differential,
    * the more accurate the distance estimate will be.
    *
    * <p>Units can be converted using the {@link edu.wpi.first.wpilibj.util.Units} class.
    *
    * @param cameraHeightMeters The physical height of the camera off the floor in meters.
    * @param targetHeightMeters The physical height of the target off the floor in meters. This
    *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
    *     top, this should be the height of the top of the target).
    * @param cameraPitchRadians The pitch of the camera from the horizontal plane in radians.
    *     Positive values up.
    * @param targetPitchRadians The pitch of the target in the camera's lens in radians. 
    *     Positive values up.
    * @return The estimated distance to the target in meters.
    */

    public static double calculateDistanceToTargetMeters(
            double cameraHeightMeters,
            double targetHeightMeters,
            double cameraPitchRadians,
            double targetPitchRadians) {
        return (targetHeightMeters - cameraHeightMeters)
                / Math.tan(cameraPitchRadians + targetPitchRadians);
    }

    /**
    * Estimate the {@link Translation2d} of the target relative to the camera.
    *
    * @param targetDistanceMeters The distance to the target in meters.
    * @param yaw The observed yaw of the target.
    * @return The target's camera-relative translation.
    */
    public static Translation2d estimateCameraToTargetTranslation(
            double targetDistanceMeters, Rotation2d yaw) {
        return new Translation2d(
                yaw.getCos() * targetDistanceMeters, yaw.getSin() * targetDistanceMeters);
    }

    /**
    * Estimate the position of the robot in the field.
    *
    * @param cameraHeightMeters The physical height of the camera off the floor in meters.
    * @param targetHeightMeters The physical height of the target off the floor in meters. This
    *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
    *     top, this should be the height of the top of the target).
    * @param cameraPitchRadians The pitch of the camera from the horizontal plane in radians.
    *     Positive values up.
    * @param targetPitchRadians The pitch of the target in the camera's lens in radians. Positive
    *     values up.
    * @param targetYaw The observed yaw of the target. Note that this *must* be CCW-positive, and
    *     Photon returns CW-positive.
    * @param gyroAngle The current robot gyro angle, likely from odometry.
    * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
    * @param cameraToRobot The position of the robot relative to the camera. If the camera was
    *     mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    *     Transform2d(3 inches, 0 inches, 0 degrees).
    * @return The position of the robot in the field.
    */
    public static Pose2d estimateFieldToRobot(
            double cameraHeightMeters,
            double targetHeightMeters,
            double cameraPitchRadians,
            double targetPitchRadians,
            Rotation2d targetYaw,
            Rotation2d gyroAngle,
            Pose2d fieldToTarget,
            Transform2d cameraToRobot) {
        return Utilities.estimateFieldToRobot(
                    Utilities.estimateCameraToTarget(
                        Utilities.estimateCameraToTargetTranslation(
                                Utilities.calculateDistanceToTargetMeters(
                                        cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians),
                                targetYaw),
                        fieldToTarget,
                        gyroAngle),
                    fieldToTarget,
                    cameraToRobot);
    }

    /**
    * Estimates a {@link Transform2d} that maps the camera position to the target position, using the
    * robot's gyro. Note that the gyro angle provided *must* line up with the field coordinate system
    * -- that is, it should read zero degrees when pointed towards the opposing alliance station, and
    * increase as the robot rotates CCW.
    *
    * @param cameraToTargetTranslation A Translation2d that encodes the x/y position of the target
    *     relative to the camera.
    * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
    * @param gyroAngle The current robot gyro angle, likely from odometry.
    * @return A Transform2d that takes us from the camera to the target.
    */
    public static Transform2d estimateCameraToTarget(
            Translation2d cameraToTargetTranslation, Pose2d fieldToTarget, Rotation2d gyroAngle) {
        // This pose maps our camera at the origin out to our target, in the robot
        // reference frame
        // The translation part of this Transform2d is from the above step, and the
        // rotation uses our robot's
        // gyro.
        return new Transform2d(
                cameraToTargetTranslation, gyroAngle.times(-1).minus(fieldToTarget.getRotation()));
    }

    /**
    * Estimates the pose of the robot in the field coordinate system, given the position of the
    * target relative to the camera, the target relative to the field, and the robot relative to the
    * camera.
    *
    * @param cameraToTarget The position of the target relative to the camera.
    * @param fieldToTarget The position of the target in the field.
    * @param cameraToRobot The position of the robot relative to the camera. If the camera was
    *     mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    *     Transform2d(3 inches, 0 inches, 0 degrees).
    * @return The position of the robot in the field.
    */
    public static Pose2d estimateFieldToRobot(
            Transform2d cameraToTarget, Pose2d fieldToTarget, Transform2d cameraToRobot) {
        return estimateFieldToCamera(cameraToTarget, fieldToTarget).transformBy(cameraToRobot);
    }

    /**
    * Estimates the pose of the camera in the field coordinate system, given the position of the
    * target relative to the camera, and the target relative to the field. This *only* tracks the
    * position of the camera, not the position of the robot itself.
    *
    * @param cameraToTarget The position of the target relative to the camera.
    * @param fieldToTarget The position of the target in the field.
    * @return The position of the camera in the field.
    */
    public static Pose2d estimateFieldToCamera(Transform2d cameraToTarget, Pose2d fieldToTarget) {
        var targetToCamera = cameraToTarget.inverse();
        return fieldToTarget.transformBy(targetToCamera);
    }

}
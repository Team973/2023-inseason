package frc.robot.shared;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Mathematical functions which pertain to swerve drive. */
public class SwerveMath {
  /**
   * Logical inverse of the Pose exponential from 254. Taken from team 3181.
   *
   * @param transform Pose to perform the log on.
   * @return {@link Twist2d} of the transformed pose.
   */
  public static Twist2d PoseLog(final Pose2d transform) {

    final double kEps = 1E-9;
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = transform.getRotation().getCos() - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  /**
   * Calculate the angle kV which will be multiplied by the radians per second for the feedforward.
   * Volt * seconds / degree == (maxVolts) / (maxSpeed)
   *
   * @param optimalVoltage Optimal voltage to use when calculating the angle kV.
   * @param motorFreeSpeedRPM Motor free speed in Rotations per Minute.
   * @param angleGearRatio Angle gear ratio, the amount of times the motor has to turn for the wheel
   *     rotation.
   * @return angle kV for feedforward.
   */
  public static double calculateAngleKV(
      double optimalVoltage, double motorFreeSpeedRPM, double angleGearRatio) {
    double maxAngularVelocity = 360 * (motorFreeSpeedRPM / angleGearRatio) / 60; // deg/s
    return optimalVoltage / maxAngularVelocity;
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desiredState Desired {@link SwerveModuleState2} to achieve.
   * @param currentAngle Current angle as a {@link Rotation2d}.
   * @param lastState The last {@link SwerveModuleState2} of the module.
   * @param moduleSteerFeedForwardClosedLoop The module feed forward closed loop for the angle
   *     motor.
   * @return Optimized {@link SwerveModuleState2}
   */
  public static SwerveModuleState2 optimize(
      SwerveModuleState2 desiredState,
      Rotation2d currentAngle,
      SwerveModuleState2 lastState,
      double moduleSteerFeedForwardClosedLoop) {
    if (moduleSteerFeedForwardClosedLoop == 0) {
      return new SwerveModuleState2(
          SwerveModuleState.optimize(desiredState, currentAngle), new Rotation2d());
    } else {
      var secondOrderOffset =
          lastState.omegaRotationPerSecond.times(moduleSteerFeedForwardClosedLoop).times(0.065);
      double targetAngle =
          SwerveMath.placeInAppropriate0To360Scope(
              currentAngle.getDegrees(), desiredState.angle.plus(secondOrderOffset).getDegrees());
      double targetSpeed = desiredState.speedMetersPerSecond;
      double delta = targetAngle - currentAngle.getDegrees();
      if (Math.abs(delta) > 90) {
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
      }
      return new SwerveModuleState2(
          targetSpeed, Rotation2d.fromDegrees(targetAngle), desiredState.omegaRotationPerSecond);
    }
  }

  /**
   * Put an angle within the 360 deg scope of a reference. For example, given a scope reference of
   * 756 degrees, assumes the full scope is (720-1080), and places an angle of 22 degrees into it,
   * returning 742 deg.
   *
   * @param scopeReference Current Angle (deg)
   * @param newAngle Target Angle (deg)
   * @return Closest angle within scope (deg)
   */
  public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  /**
   * Perform anti-jitter within modules if the speed requested is too low.
   *
   * @param moduleState Current {@link SwerveModuleState2} requested.
   * @param lastModuleState Previous {@link SwerveModuleState2} used.
   * @param maxSpeed Maximum speed of the modules, should be in {@link
   *     SwerveDriveConfiguration#maxSpeed}.
   */
  public static void antiJitter(
      SwerveModuleState2 moduleState, SwerveModuleState2 lastModuleState, double maxSpeed) {
    if (Math.abs(moduleState.speedMetersPerSecond) <= (maxSpeed * 0.01)) {
      moduleState.angle = lastModuleState.angle;
      moduleState.omegaRotationPerSecond = lastModuleState.omegaRotationPerSecond;
    }
  }
}

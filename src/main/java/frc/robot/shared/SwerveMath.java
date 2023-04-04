package frc.robot.shared;

import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveMath {
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
   * Optimize the angle of the {@link SwerveModuleState2} to be the closest angle to the current
   * angle. Taken from <a
   * href="https://github.com/pittsfordrobotics/REVSwerve2023/blob/master/src/main/java/com/team3181/lib/swerve/SwerveOptimizer.java">Team
   * 3181.</a>
   *
   * @param desiredState Desired {@link SwerveModuleState2} to achieve.
   * @param currentAngle Current angle as a {@link Rotation2d}.
   * @param secondOrderOffsetDegrees Offset calculated using 2nd order kinematics.
   * @return Optimized {@link SwerveModuleState2}
   */
  public static SwerveModuleState2 optimize(
      SwerveModuleState2 desiredState, Rotation2d currentAngle, double secondOrderOffsetDegrees) {
    double targetAngle =
        SwerveMath.placeInAppropriate0To360Scope(
            currentAngle.getDegrees(), desiredState.angle.getDegrees() + secondOrderOffsetDegrees);
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      if (delta > 90) {
        targetAngle -= 180;
      } else {
        targetAngle += 180;
      }
    }
    // Ensure outputted angle is positive.
    while (targetAngle < 0) {
      targetAngle += 360;
    }
    return new SwerveModuleState2(
        targetSpeed, Rotation2d.fromDegrees(targetAngle), desiredState.omegaRadPerSecond);
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
      moduleState.omegaRadPerSecond = 0;
    }
  }
}

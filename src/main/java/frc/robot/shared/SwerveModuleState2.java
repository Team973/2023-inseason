package frc.robot.shared;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.AllArgsConstructor;

/** Second order kinematics swerve module state. */
@AllArgsConstructor
public class SwerveModuleState2 extends SwerveModuleState {
  /** Swerve module speed in meters per second. */
  public double speedMetersPerSecond;
  /** Swerve module angle as a {@link Rotation2d}. */
  public Rotation2d angle;
  /** Swerve module rotation per sec as a {@link Rotation2d}. */
  public Rotation2d omegaRotationPerSecond;

  public SwerveModuleState2() {
    speedMetersPerSecond = 0;
    angle = new Rotation2d();
    omegaRotationPerSecond = new Rotation2d();
  }
}

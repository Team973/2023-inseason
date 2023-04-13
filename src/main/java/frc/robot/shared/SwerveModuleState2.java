package frc.robot.shared;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Second order kinematics swerve module state. */
public class SwerveModuleState2 extends SwerveModuleState {
  public Rotation2d omegaRotationPerSecond;

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public SwerveModuleState2() {
    super();
    this.omegaRotationPerSecond = new Rotation2d();
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   * @param omegaRadPerSecond The angular velocity of the module.
   */
  public SwerveModuleState2(
      double speedMetersPerSecond, Rotation2d angle, Rotation2d omegaRotationPerSecond) {
    super(speedMetersPerSecond, angle);
    this.omegaRotationPerSecond = omegaRotationPerSecond;
  }

  /**
   * Create a {@link SwerveModuleState2} based on the {@link SwerveModuleState} with the radians per
   * second defined.
   *
   * @param state First order kinematic module state.
   * @param omegaRadPerSecond Module wheel angular rotation in radians per second.
   */
  public SwerveModuleState2(SwerveModuleState state, Rotation2d omegaRotationPerSecond) {
    super(state.speedMetersPerSecond, state.angle);
    this.omegaRotationPerSecond = omegaRotationPerSecond;
  }

  /**
   * Convert to a {@link SwerveModuleState}.
   *
   * @return {@link SwerveModuleState} with the same angle and speed.
   */
  public SwerveModuleState toSwerveModuleState() {
    return new SwerveModuleState(this.speedMetersPerSecond, this.angle);
  }
}

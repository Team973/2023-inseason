package frc.robot.devices;

import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.Rotation3d2;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class GreyPigeon {
  private final Pigeon2 m_pigeon;
  private Rotation3d2 m_offset;

  private static final double LEVEL_TOLERANCE_DEG = 2.0;

  /** Creates a new Gyro. */
  public GreyPigeon() {
    m_pigeon = new Pigeon2(DriveInfo.PIGEON_ID, RobotInfo.CANIVORE_NAME);
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());

    reset();
  }

  public Rotation3d2 getRotation() {
    return new Rotation3d2(getYaw(), getPitch(), getRoll());
  }

  /**
   * Returns the yaw from the Pigeon IMU with the offset applied.
   *
   * @return The yaw from the Pigeon IMU with the offset applied.
   */
  public Rotation2d getYaw() {
    return getRawYaw().minus(m_offset.getYaw());
  }

  /**
   * Returns the pitch from the Pigeon IMU with the offset applied.
   *
   * @return The pitch from the Pigeon IMU with the offset applied.
   */
  public Rotation2d getPitch() {
    return getRawPitch().minus(m_offset.getPitch());
  }

  /**
   * Returns the roll from the Pigeon IMU with the offset applied.
   *
   * @return The roll from the Pigeon IMU with the offset applied.
   */
  public Rotation2d getRoll() {
    return getRawRoll().minus(m_offset.getRoll());
  }

  /**
   * Returns the raw yaw from the Pigeon IMU.
   *
   * @return The raw yaw from the Pigeon IMU.
   */
  public Rotation2d getRawYaw() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw().getValue());
  }

  /**
   * Returns the raw pitch from the Pigeon IMU.
   *
   * @return The raw pitch from the Pigeon IMU.
   */
  public Rotation2d getRawPitch() {
    return Rotation2d.fromDegrees(m_pigeon.getPitch().getValue());
  }

  /**
   * Returns the raw roll from the Pigeon IMU.
   *
   * @return The raw roll from the Pigeon IMU.
   */
  public Rotation2d getRawRoll() {
    return Rotation2d.fromDegrees(m_pigeon.getRoll().getValue());
  }

  /**
   * Returns the normalized yaw from the Pigeon IMU with the offset applied.
   *
   * @return The normalized yaw from the Pigeon IMU with the offset applied.
   */
  public Rotation2d getNormalizedYaw() {
    double rawYaw = getYaw().getDegrees();
    double normalizedYaw = Math.IEEEremainder(rawYaw, 360.0);
    if (normalizedYaw < 0) {
      normalizedYaw += 360.0;
    }
    return Rotation2d.fromDegrees(normalizedYaw);
  }

  /**
   * Returns whether the Pigeon IMU is level.
   *
   * @return Whether the Pigeon IMU is level.
   */
  public boolean isLevel() {
    double pitch = getPitch().getDegrees();
    double roll = getRoll().getDegrees();

    return Math.abs(pitch) <= LEVEL_TOLERANCE_DEG && Math.abs(roll) <= LEVEL_TOLERANCE_DEG;
  }

  /** Resets the offset to the current yaw, pitch, and roll. */
  public void reset() {
    double yawRad = getRawYaw().getRadians();
    double pitchRad = getRawPitch().getRadians();
    double rollRad = getRawRoll().getRadians();
    m_offset = new Rotation3d2(yawRad, pitchRad, rollRad);
  }
}

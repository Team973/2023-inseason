package frc.robot.devices;

import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.StandardizedRotation3d;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/** GreyPigeon - Pigeon2 wrapper class */
public class GreyPigeon {
  private final Pigeon2 m_pigeon;
  private StandardizedRotation3d m_offset;

  private static final double DEFAULT_LEVEL_TOLERANCE_DEG = 2.0;

  /** Creates a new GreyPigeon. */
  public GreyPigeon() {
    m_pigeon = new Pigeon2(DriveInfo.PIGEON_ID, RobotInfo.CANIVORE_NAME);
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());

    reset();
  }

  /**
   * Returns a StandardizedRotation3d object containing the yaw, pitch, and roll from the Pigeon2.
   *
   * @return A StandardizedRotation3d object containing the adjusted yaw, pitch, and roll from the
   *     Pigeon2.
   */
  public StandardizedRotation3d getRotation() {
    return getRawRotation().minus(m_offset);
  }

  /**
   * Returns a StandardizedRotation3d object containing the raw yaw, pitch, and roll from the
   * Pigeon2.
   *
   * @return A StandardizedRotation3d object containing the raw yaw, pitch, and roll from the
   *     Pigeon2.
   */
  public StandardizedRotation3d getRawRotation() {
    return new StandardizedRotation3d(getRawRoll(), getRawPitch(), getRawYaw());
  }

  /**
   * Returns the yaw from the Pigeon2 with the offset applied.
   *
   * @return The yaw from the Pigeon2 with the offset applied.
   */
  public Rotation2d getYaw() {
    System.out.println(m_offset.getYaw().getDegrees());
    return getRawYaw().minus(m_offset.getYaw());
  }

  /**
   * Returns the pitch from the Pigeon2 with the offset applied.
   *
   * @return The pitch from the Pigeon2 with the offset applied.
   */
  public Rotation2d getPitch() {
    return getRawPitch().minus(m_offset.getPitch());
  }

  /**
   * Returns the roll from the Pigeon2 with the offset applied.
   *
   * @return The roll from the Pigeon2 with the offset applied.
   */
  public Rotation2d getRoll() {
    return getRawRoll().minus(m_offset.getRoll());
  }

  /**
   * Returns the raw yaw from the Pigeon2.
   *
   * @return The raw yaw from the Pigeon2.
   */
  public Rotation2d getRawYaw() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw().getValue());
  }

  /**
   * Returns the raw pitch from the Pigeon2.
   *
   * @return The raw pitch from the Pigeon2.
   */
  public Rotation2d getRawPitch() {
    return Rotation2d.fromDegrees(m_pigeon.getPitch().getValue());
  }

  /**
   * Returns the raw roll from the Pigeon2.
   *
   * @return The raw roll from the Pigeon2.
   */
  public Rotation2d getRawRoll() {
    return Rotation2d.fromDegrees(m_pigeon.getRoll().getValue());
  }

  /**
   * Returns the normalized yaw from the Pigeon2 with the offset applied.
   *
   * @return The normalized yaw from the Pigeon2 with the offset applied.
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
   * Returns whether the Pigeon2 is level.
   *
   * @return Whether the Pigeon2 is level.
   */
  public boolean isLevel() {
    return isLevel(DEFAULT_LEVEL_TOLERANCE_DEG);
  }

  /**
   * Returns whether the Pigeon2 is level within the specified tolerance.
   *
   * @param tolerance The tolerance in degrees.
   * @return Whether the Pigeon2 is level within the specified tolerance.
   */
  public boolean isLevel(double tolerance) {
    double pitch = getPitch().getDegrees();
    double roll = getRoll().getDegrees();

    return Math.abs(pitch) <= tolerance && Math.abs(roll) <= tolerance;
  }

  /** Resets the offset to the current yaw, pitch, and roll. */
  public void reset() {
    m_offset = getRawRotation();
    System.out.println(m_offset);
  }
}

package frc.robot.shared;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class Rotation3d2 extends Rotation3d {

  /** Constructs a Rotation3d with a default angle of 0 degrees. */
  public Rotation3d2() {
    super();
  }

  /**
   * Constructs a Rotation3d2 from a Rotation3d.
   *
   * @param rotation3d The Rotation3d to copy.
   */
  public Rotation3d2(Rotation3d rotation3d) {
    this(rotation3d.getQuaternion());
  }

  /**
   * Constructs a Rotation3d from three Rotation2d.
   *
   * @param x The Rotation2d around the x axis (roll).
   * @param y The Rotation2d around the y axis (pitch).
   * @param z The Rotation2d around the z axis (yaw).
   */
  public Rotation3d2(Rotation2d x, Rotation2d y, Rotation2d z) {
    this(x.getRadians(), y.getRadians(), z.getRadians());
  }

  /**
   * Constructs a Rotation3d from a quaternion.
   *
   * @param q The quaternion.
   */
  @JsonCreator
  public Rotation3d2(@JsonProperty(required = true, value = "quaternion") Quaternion q) {
    super(q);
  }

  /**
   * Constructs a Rotation3d from extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll) in radians.
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch) in radians.
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw) in radians.
   */
  public Rotation3d2(double roll, double pitch, double yaw) {
    super(roll, pitch, yaw);
  }

  /**
   * Constructs a Rotation3d with the given rotation vector representation. This representation is
   * equivalent to axis-angle, where the normalized axis is multiplied by the rotation around the
   * axis in radians.
   *
   * @param rvec The rotation vector.
   */
  public Rotation3d2(Vector<N3> rvec) {
    super(rvec);
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angleRadians The rotation around the axis in radians.
   */
  public Rotation3d2(Vector<N3> axis, double angleRadians) {
    super(axis, angleRadians);
  }

  /**
   * Constructs a Rotation3d from a rotation matrix.
   *
   * @param rotationMatrix The rotation matrix.
   * @throws IllegalArgumentException if the rotation matrix isn't special orthogonal.
   */
  public Rotation3d2(Matrix<N3, N3> rotationMatrix) {
    super(rotationMatrix);
  }

  /**
   * Constructs a Rotation3d that rotates the initial vector onto the final vector.
   *
   * <p>This is useful for turning a 3D vector (final) into an orientation relative to a coordinate
   * system vector (initial).
   *
   * @param initial The initial vector.
   * @param last The final vector.
   */
  public Rotation3d2(Vector<N3> initial, Vector<N3> last) {
    super(initial, last);
  }

  /**
   * Get the Rotation2d representing the yaw of the rotation.
   *
   * @return The Rotation2d representing the yaw of the rotation.
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(getZ());
  }

  /**
   * Get the Rotation2d representing the pitch of the rotation.
   *
   * @return The Rotation2d representing the pitch of the rotation.
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(getX());
  }

  /**
   * Get the Rotation2d representing the roll of the rotation.
   *
   * @return The Rotation2d representing the roll of the rotation.
   */
  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(getY());
  }

  @Override
  public Rotation3d2 plus(Rotation3d other) {
    return new Rotation3d2(super.plus(other));
  }

  @Override
  public Rotation3d2 minus(Rotation3d other) {
    return new Rotation3d2(super.minus(other));
  }

  @Override
  public Rotation3d unaryMinus() {
    return new Rotation3d2(super.unaryMinus());
  }

  @Override
  public Rotation3d times(double scalar) {
    return new Rotation3d2(super.times(scalar));
  }

  @Override
  public Rotation3d2 div(double scalar) {
    return new Rotation3d2(super.div(scalar));
  }

  @Override
  public Rotation3d2 rotateBy(Rotation3d other) {
    return new Rotation3d2(other.getQuaternion().times(getQuaternion()));
  }

  @Override
  public String toString() {
    return String.format("Rotation3d2(%s)", getQuaternion());
  }

  @Override
  public Rotation3d interpolate(Rotation3d endValue, double t) {
    return new Rotation3d2(super.interpolate(endValue, t));
  }
}

package frc.robot.subsystems.swerve;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.ejml.simple.SimpleMatrix;

/**
 * Clone of WPI SwerveKinematics, which implements second order kinematics when calculating modules
 * states from chassis speed.
 *
 * <p>
 *
 * <p>Makes use of {@link SwerveModuleState2} to add the angular velocity that is required of the
 * module as an output.
 */
public class SwerveKinematics2 extends SwerveDriveKinematics {
  private final SimpleMatrix m_inverseKinematics;
  private final SimpleMatrix bigInverseKinematics;

  private final int m_numModules;
  private final Translation2d[] m_modules;
  private final SwerveModuleState2[] m_moduleStates;
  private Translation2d m_prevCoR = new Translation2d();

  /**
   * Constructs a swerve drive kinematics object. This takes in a variable number of wheel locations
   * as Translation2ds. The order in which you pass in the wheel locations is the same order that
   * you will receive the module states when performing inverse kinematics. It is also expected that
   * you pass in the module states in the same order when calling the forward kinematics methods.
   *
   * @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
   */
  public SwerveKinematics2(Translation2d... wheelsMeters) {
    super(wheelsMeters);

    m_numModules = wheelsMeters.length;
    m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
    m_moduleStates = new SwerveModuleState2[m_numModules];
    Arrays.fill(m_moduleStates, new SwerveModuleState2());
    m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);
    bigInverseKinematics = new SimpleMatrix(m_numModules * 2, 4);

    for (int i = 0; i < m_numModules; i++) {
      m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
      m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
      bigInverseKinematics.setRow(
          i * 2, 0, /* Start Data */ 1, 0, -m_modules[i].getX(), -m_modules[i].getY());
      bigInverseKinematics.setRow(
          i * 2 + 1, 0, /* Start Data */ 0, 1, -m_modules[i].getY(), +m_modules[i].getX());
    }
  }

  /**
   * Performs inverse kinematics to return the module states from a desired chassis velocity. This
   * method is often used to convert joystick values into module speeds and angles.
   *
   * <p>This function also supports variable centers of rotation. During normal operations, the
   * center of rotation is usually the same as the physical center of the robot; therefore, the
   * argument is defaulted to that use case. However, if you wish to change the center of rotation
   * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
   *
   * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary),
   * the previously calculated module angle will be maintained.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
   *     component, the robot will rotate around that corner.
   * @return An array containing the module states. Use caution because these module states are not
   *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
   *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SwerveModuleState2[],
   *     double) DesaturateWheelSpeeds} function to rectify this issue.
   */
  @SuppressWarnings("PMD.MethodReturnsInternalArray")
  public SwerveModuleState2[] to2ndOrderSwerveModuleStates(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
    if (chassisSpeeds.vxMetersPerSecond == 0.0
        && chassisSpeeds.vyMetersPerSecond == 0.0
        && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
      for (int i = 0; i < m_numModules; i++) {
        m_moduleStates[i].speedMetersPerSecond = 0.0;
      }

      return m_moduleStates;
    }

    if (!centerOfRotationMeters.equals(m_prevCoR)) {
      for (int i = 0; i < m_numModules; i++) {
        m_inverseKinematics.setRow(
            i * 2, 0, /* Start Data */ 1, 0, -m_modules[i].getY() + centerOfRotationMeters.getY());
        m_inverseKinematics.setRow(
            i * 2 + 1,
            0, /* Start Data */
            0,
            1,
            +m_modules[i].getX() - centerOfRotationMeters.getX());
        bigInverseKinematics.setRow(
            i * 2,
            0, /* Start Data */
            1,
            0,
            -m_modules[i].getX() + centerOfRotationMeters.getX(),
            -m_modules[i].getY() + centerOfRotationMeters.getY());
        bigInverseKinematics.setRow(
            i * 2 + 1,
            0, /* Start Data */
            0,
            1,
            -m_modules[i].getY() + centerOfRotationMeters.getY(),
            +m_modules[i].getX() - centerOfRotationMeters.getX());
      }
      m_prevCoR = centerOfRotationMeters;
    }

    var chassisSpeedsVector = new SimpleMatrix(3, 1);
    chassisSpeedsVector.setColumn(
        0,
        0,
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);

    var moduleVelocityStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);

    var accelerationVector = new SimpleMatrix(4, 1);
    accelerationVector.setColumn(0, 0, 0, 0, Math.pow(chassisSpeeds.omegaRadiansPerSecond, 2), 0);

    var moduleAccelerationStatesMatrix = bigInverseKinematics.mult(accelerationVector);

    for (int i = 0; i < m_numModules; i++) {
      double x = moduleVelocityStatesMatrix.get(i * 2, 0);
      double y = moduleVelocityStatesMatrix.get(i * 2 + 1, 0);

      double ax = moduleAccelerationStatesMatrix.get(i * 2, 0);
      double ay = moduleAccelerationStatesMatrix.get(i * 2 + 1, 0);

      double speed = Math.hypot(x, y);
      Rotation2d angle = new Rotation2d(x, y);

      var trigThetaAngle = new SimpleMatrix(2, 2);
      trigThetaAngle.setColumn(0, 0, angle.getCos(), -angle.getSin());
      trigThetaAngle.setColumn(1, 0, angle.getSin(), angle.getCos());

      var accelVector = new SimpleMatrix(2, 1);
      accelVector.setColumn(0, 0, ax, ay);

      var omegaVector = trigThetaAngle.mult(accelVector);

      double omega = (omegaVector.get(1, 0) / speed) - chassisSpeeds.omegaRadiansPerSecond;
      m_moduleStates[i] = new SwerveModuleState2(speed, angle, omega);
    }

    return m_moduleStates;
  }

  /**
   * Performs inverse kinematics. See {@link #to2ndOrderSwerveModuleStates(ChassisSpeeds,
   * Translation2d)} to2ndOrderSwerveModuleStates for more information.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @return An array containing the module states.
   */
  public SwerveModuleState2[] to2ndOrderSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
    return to2ndOrderSwerveModuleStates(chassisSpeeds, new Translation2d());
  }
}

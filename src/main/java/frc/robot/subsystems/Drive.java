package frc.robot.subsystems;

import frc.robot.devices.GreyPigeon;
import frc.robot.greydash.GreyDashClient;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.Subsystem;
import frc.robot.shared.SwerveDriveKinematics2;
import frc.robot.shared.SwerveDrivePoseEstimator2;
import frc.robot.shared.SwerveMath;
import frc.robot.shared.SwerveModuleState2;
import frc.robot.subsystems.swerve.SwerveModule;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Drive implements Subsystem {
  private static final Rotation2d BALANCE_CUTOFF_THRESHOLD = Rotation2d.fromDegrees(6.5);

  private static final boolean ENABLE_CHASSIS_VELOCITY_CORRECTION = true;
  private static final boolean ENABLE_HEADING_CORRECTION = true;

  private SwerveDrivePoseEstimator2 m_swerveDrivePoseEstimator;
  private final SwerveModule[] m_swerveModules;
  private ChassisSpeeds m_currentChassisSpeeds;
  @Getter private final GreyPigeon m_pigeon;

  @Setter private Rotation2d m_targetRobotAngle = new Rotation2d();

  /**
   * Trustworthiness of the internal model of how motors should be moving Measured in expected
   * standard deviation (meters of position and degrees of rotation)
   */
  public Matrix<N3, N1> m_stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  /**
   * Trustworthiness of the vision system Measured in expected standard deviation (meters of
   * position and degrees of rotation)
   */
  public Matrix<N3, N1> m_visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

  @Setter private RotationControl m_rotationControl = RotationControl.OpenLoop;

  private final PIDController m_rotationController = new PIDController(0.11, 0.0, 0.003);
  private final PIDController m_balancePitchController = new PIDController(0.055, 0.0, 0.015);
  private final PIDController m_balanceRollController = new PIDController(0.055, 0.0, 0.015);

  public enum RotationControl {
    OpenLoop,
    ClosedLoop,
  }

  public static class AnglePresets {
    public static final Rotation2d TOWARDS_HP = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d TOWARDS_DS = Rotation2d.fromDegrees(180.0);
    public static final Rotation2d TOWARDS_MHP_RED = Rotation2d.fromDegrees(-90.0);
    public static final Rotation2d TOWARDS_MHP_BLUE = Rotation2d.fromDegrees(90.0);
    public static final Rotation2d TOWARDS_WS_BLUE = Rotation2d.fromDegrees(45.0);
    public static final Rotation2d TOWARDS_WSR_BLUE = Rotation2d.fromDegrees(135.0);
    public static final Rotation2d TOWARDS_WS_RED = Rotation2d.fromDegrees(-45.0);
    public static final Rotation2d TOWARDS_WSR_RED = Rotation2d.fromDegrees(-135.0);
  }

  private final HolonomicDriveController m_controller;

  public Drive(GreyPigeon pigeon) {
    m_pigeon = pigeon;

    m_swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, DriveInfo.FRONT_LEFT_CONSTANTS),
          new SwerveModule(1, DriveInfo.FRONT_RIGHT_CONSTANTS),
          new SwerveModule(2, DriveInfo.BACK_LEFT_CONSTANTS),
          new SwerveModule(3, DriveInfo.BACK_RIGHT_CONSTANTS)
        };

    m_currentChassisSpeeds = new ChassisSpeeds();

    m_swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator2(
            DriveInfo.SWERVE_KINEMATICS,
            m_pigeon.getYaw(),
            getPositions(),
            new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
            m_stateStdDevs,
            m_visionMeasurementStdDevs);

    m_controller =
        new HolonomicDriveController(
            new PIDController(1.3, 0.0, 0.0),
            new PIDController(1.3, 0.0, 0.0),
            new ProfiledPIDController(
                10.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 7.0)));

    // Used by balanceDrive() to keep the robot level on the charge station
    m_balancePitchController.setSetpoint(0.0);
    m_balanceRollController.setSetpoint(0.0);
    m_balancePitchController.setTolerance(5.0);
    m_balanceRollController.setTolerance(5.0);
  }

  /** Balance the robot on the charge station */
  public void balanceDrive() {
    var pitch = m_pigeon.getPitch();
    var roll = m_pigeon.getRoll();
    var yaw = m_pigeon.getNormalizedYaw();

    // Determine the sign for pitch and roll based on the yaw angle
    double pitchSign = Math.signum(yaw.getCos());
    double rollSign = Math.signum(yaw.getSin());

    // Calculate the pitch and roll output using the determined signs
    double pitchOutput =
        !m_pigeon.isLevel(BALANCE_CUTOFF_THRESHOLD)
            ? m_balancePitchController.calculate(pitch.getDegrees(), 0.0) * pitchSign
            : 0.0;

    double rollOutput =
        !m_pigeon.isLevel(BALANCE_CUTOFF_THRESHOLD)
            ? m_balanceRollController.calculate(roll.getDegrees(), 0.0) * rollSign
            : 0.0;

    // Apply the translation to the holonomic drive with a zero rotation value
    driveInput(new Translation2d(pitchOutput + rollOutput, 0.0), 0.0);
  }

  /**
   * Drive the robot using the given translation and rotation values
   *
   * @param translation The translation to drive in
   * @param rotationVal The rotation to drive in
   */
  public void driveInput(Translation2d translation, double rotationVal) {
    double rotation = rotationVal;
    var speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, m_pigeon.getYaw());

    // Thank you to Jared Russell FRC254 for Open Loop Compensation Code
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    if (ENABLE_CHASSIS_VELOCITY_CORRECTION) {
      double dtConstant = 0.009;
      Pose2d robotPoseVel =
          new Pose2d(
              speeds.vxMetersPerSecond * dtConstant,
              speeds.vyMetersPerSecond * dtConstant,
              Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dtConstant));
      Twist2d twistVel = SwerveMath.PoseLog(robotPoseVel);

      speeds =
          new ChassisSpeeds(
              twistVel.dx / dtConstant, twistVel.dy / dtConstant, twistVel.dtheta / dtConstant);
    }

    // Heading correction
    if (ENABLE_HEADING_CORRECTION) {
      final Rotation2d currentYaw = m_pigeon.getNormalizedYaw();
      if (m_rotationControl == RotationControl.ClosedLoop) {
        double diff = m_targetRobotAngle.minus(currentYaw).getDegrees();
        if (diff > 180) {
          diff -= 360;
        } else if (diff < -180) {
          diff += 360;
        }

        speeds.omegaRadiansPerSecond =
            m_rotationController.calculate(currentYaw.getDegrees(), currentYaw.getDegrees() + diff)
                * DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      }
    }

    m_currentChassisSpeeds = speeds;
  }

  /**
   * Drive the robot using the given trajectory state and rotation values.
   *
   * @param state The trajectory state to drive in.
   * @param rotation The rotation to drive in.
   */
  public void driveInput(State state, Rotation2d rotation) {
    m_currentChassisSpeeds = m_controller.calculate(getPose(), state, rotation);
  }

  /**
   * Set the desired module states to the swerve modules.
   *
   * @param desiredStates The desired module states.
   */
  public void setModuleStates(SwerveModuleState2[] desiredStates) {
    SwerveDriveKinematics2.desaturateWheelSpeeds(
        desiredStates, DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);

    double states[] = new double[8];
    int index = 0;
    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
      states[index] = desiredStates[mod.moduleNumber].angle.getDegrees();
      states[index + 1] = desiredStates[mod.moduleNumber].speedMetersPerSecond;
      index += 2;
    }
  }

  /**
   * Get the current robot pose estimation.
   *
   * @return The current robot pose.
   */
  public Pose2d getPose() {
    return m_swerveDrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link
   * SwerveIMU} gyro reading with the given timestamp of the vision measurement.
   *
   * @param robotPose Robot {@link Pose2d} as measured by vision.
   * @param timestamp Timestamp the measurement was taken as time since startup, should be taken
   *     from {@link Timer#getFPGATimestamp()} or similar sources.
   * @param soft Add vision estimate using the {@link
   *     SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)} function, or hard reset
   *     odometry with the given position with {@link
   *     edu.wpi.first.math.kinematics.SwerveDriveOdometry#resetPosition(Rotation2d,
   *     SwerveModulePosition[], Pose2d)}.
   * @param trustWorthiness Trust level of vision reading when using a soft measurement, used to
   *     multiply the standard deviation. Set to 1 for full trust.
   */
  public void addVisionMeasurement(
      Pose2d robotPose, double timestamp, boolean soft, double trustWorthiness) {
    if (soft) {
      m_swerveDrivePoseEstimator.addVisionMeasurement(
          robotPose, timestamp, m_visionMeasurementStdDevs.times(1.0 / trustWorthiness));
    } else {
      m_swerveDrivePoseEstimator.resetPosition(robotPose.getRotation(), getPositions(), robotPose);
    }
  }

  /**
   * Reset the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_swerveDrivePoseEstimator.resetPosition(m_pigeon.getYaw(), getPositions(), pose);
  }

  /** Reset the module encoders to the current absolute position. */
  public void resetModules() {
    for (var mod : m_swerveModules) {
      mod.resetToAbsolute();
    }
  }

  private SwerveModulePosition[] getPositions() {
    var positions = new SwerveModulePosition[4];
    for (SwerveModule mod : m_swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void enableBrakeMode() {
    for (var mod : m_swerveModules) {
      mod.driveBrake();
    }
  }

  public void disableBrakeMode() {
    for (var mod : m_swerveModules) {
      mod.driveNeutral();
    }
  }

  public void dashboardUpdate() {
    GreyDashClient.setGyroAngle(m_pigeon.getYaw().getDegrees());
  }

  public void debugDashboardUpdate() {
    double states[] = new double[8];
    int index = 0;

    for (SwerveModule mod : m_swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Drive Stator", mod.getDriveStatorCurrent());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Drive Supply", mod.getDriveSupplyCurrent());
      states[index] = mod.getState().angle.getDegrees();
      states[index + 1] = mod.getState().speedMetersPerSecond;
      index += 2;
    }

    SmartDashboard.putNumberArray("swerve/actual", states);
    SmartDashboard.putNumber("pitch", m_pigeon.getPitch().getDegrees());
    SmartDashboard.putNumber("roll", m_pigeon.getRoll().getDegrees());
    SmartDashboard.putNumberArray(
        "swerve/odometry",
        ImmutableList.of(
                getPose().getTranslation().getX(),
                getPose().getTranslation().getY(),
                getPose().getRotation().getDegrees())
            .toArray(Double[]::new));
  }

  public void update() {
    m_swerveDrivePoseEstimator.update(
        m_pigeon.getYaw(), m_pigeon.getPitch(), m_pigeon.getRoll(), getPositions());

    Pose2d robot_pose_vel =
        new Pose2d(
            m_currentChassisSpeeds.vxMetersPerSecond * 0.03,
            m_currentChassisSpeeds.vyMetersPerSecond * 0.03,
            new Rotation2d(m_currentChassisSpeeds.omegaRadiansPerSecond * 0.03));
    Pose2d robot_cur_pose = new Pose2d();
    Twist2d twist_vel = robot_cur_pose.log(robot_pose_vel);
    ChassisSpeeds updated_chassis_speeds =
        new ChassisSpeeds(twist_vel.dx / 0.03, twist_vel.dy / 0.03, twist_vel.dtheta / 0.03);

    SwerveModuleState2[] swerveModuleStates =
        DriveInfo.SWERVE_KINEMATICS.toSwerveModuleStates(updated_chassis_speeds);

    setModuleStates(swerveModuleStates);
  }

  public void reset() {
    m_pigeon.reset();
    m_targetRobotAngle = m_pigeon.getYaw();
    resetOdometry(new Pose2d());
  }
}

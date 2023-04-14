package frc.robot.subsystems;

import frc.robot.devices.GreyPigeon;
import frc.robot.greydash.GreyDashClient;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.Subsystem;
import frc.robot.subsystems.swerve.SwerveModule;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Drive implements Subsystem {
  private static final Rotation2d BALANCE_CUTOFF_THRESHOLD = Rotation2d.fromDegrees(6.0);

  private static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(DriveInfo.TRACKWIDTH_METERS / 2.0, DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(DriveInfo.TRACKWIDTH_METERS / 2.0, -DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(-DriveInfo.TRACKWIDTH_METERS / 2.0, DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(-DriveInfo.TRACKWIDTH_METERS / 2.0, -DriveInfo.WHEELBASE_METERS / 2.0)
  };

  private final SwerveDriveOdometry m_swerveOdometry;
  private final SwerveModule[] m_swerveModules;
  private ChassisSpeeds m_currentChassisSpeeds;

  @Getter private final GreyPigeon m_pigeon;

  @Setter private Rotation2d m_targetRobotAngle = new Rotation2d();
  @Setter private RotationControl m_rotationControl = RotationControl.OpenLoop;

  private final PIDController m_rotationController = new PIDController(0.0973, 0.0, 0.001);
  private final PIDController m_balancePitchController = new PIDController(0.05, 0.0, 0.015);
  private final PIDController m_balanceRollController = new PIDController(0.05, 0.0, 0.015);

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

    m_swerveOdometry =
        new SwerveDriveOdometry(DriveInfo.SWERVE_KINEMATICS, m_pigeon.getYaw(), getPositions());

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
    driveInput(new Translation2d(pitchOutput + rollOutput, 0.0), 0.0, true);
  }

  public void driveInput(Translation2d translation, double rotationVal, boolean fieldRelative) {
    double rotation = rotationVal;
    final Rotation2d currentYaw = m_pigeon.getNormalizedYaw();
    if (m_rotationControl == RotationControl.ClosedLoop) {
      double diff = m_targetRobotAngle.minus(currentYaw).getDegrees();
      if (diff > 180) {
        diff -= 360;
      } else if (diff < -180) {
        diff += 360;
      }

      rotation =
          m_rotationController.calculate(currentYaw.getDegrees(), currentYaw.getDegrees() + diff);
    } else {
      m_targetRobotAngle = currentYaw.minus(m_pigeon.getAngularVelocity().times(0.05));
    }

    m_currentChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, m_pigeon.getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
  }

  public void driveInput(State state, Rotation2d rotation) {
    m_currentChassisSpeeds = m_controller.calculate(getPose(), state, rotation);
  }

  public void xOutModules() {
    // Side effect: any drive input that goes above the anti-jitter threshold overrides this
    int index = 0;
    for (SwerveModule mod : m_swerveModules) {
      double angleToCenter =
          Math.atan2(MODULE_LOCATIONS[index].getY(), MODULE_LOCATIONS[index].getX());
      index++;

      mod.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromRadians(angleToCenter)), true);
    }
  }

  /* Used by Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);

    double states[] = new double[8];
    int index = 0;
    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
      states[index] = desiredStates[mod.moduleNumber].angle.getDegrees();
      states[index + 1] = desiredStates[mod.moduleNumber].speedMetersPerSecond;
      index += 2;
    }

    SmartDashboard.putNumberArray("swerve/setpoints", states);
  }

  public Pose2d getPose() {
    return m_swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_pigeon.setYawOffset(pose.getRotation());
    m_swerveOdometry.resetPosition(m_pigeon.getYaw(), getPositions(), pose);
  }

  public void resetModules() {
    for (SwerveModule mod : m_swerveModules) {
      mod.resetToAbsolute();
    }
  }

  private SwerveModulePosition[] getPositions() {
    var positions = new SwerveModulePosition[4];
    for (var mod : m_swerveModules) {
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

    SmartDashboard.putNumber("Drive Angle Target", m_targetRobotAngle.getDegrees());
    SmartDashboard.putNumber("Drive Angle", m_pigeon.getYaw().getDegrees());
  }

  public void update() {
    m_swerveOdometry.update(m_pigeon.getYaw(), getPositions());

    Pose2d robot_pose_vel =
        new Pose2d(
            m_currentChassisSpeeds.vxMetersPerSecond * 0.03,
            m_currentChassisSpeeds.vyMetersPerSecond * 0.03,
            new Rotation2d(m_currentChassisSpeeds.omegaRadiansPerSecond * 0.03));
    Pose2d robot_cur_pose = new Pose2d();
    Twist2d twist_vel = robot_cur_pose.log(robot_pose_vel);
    ChassisSpeeds updated_chassis_speeds =
        new ChassisSpeeds(twist_vel.dx / 0.03, twist_vel.dy / 0.03, twist_vel.dtheta / 0.03);

    SwerveModuleState[] swerveModuleStates =
        DriveInfo.SWERVE_KINEMATICS.toSwerveModuleStates(updated_chassis_speeds);

    setModuleStates(swerveModuleStates);
  }

  public void reset() {
    m_pigeon.reset();
    m_targetRobotAngle = m_pigeon.getYaw();
    resetOdometry(new Pose2d());
  }
}

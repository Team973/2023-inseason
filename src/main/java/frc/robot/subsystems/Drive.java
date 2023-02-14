package frc.robot.subsystems;

import frc.robot.greydash.GreyDashClient;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.Subsystem;
import frc.robot.subsystems.swerve.SwerveModule;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Drive implements Subsystem {
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] m_swerveModules;

  private final Pigeon2 m_pigeon;
  private double m_gyroOffsetDegrees;

  @Setter private double m_targetRobotAngle = 0.0;

  @Setter private RotationControl m_rotationControl = RotationControl.OpenLoop;

  private PIDController m_rotationController = new PIDController(0.1125, 0.0, 0.003);

  public enum RotationControl {
    OpenLoop,
    ClosedLoop,
  }

  public static class AnglePresets {
    public static final double TOWARDS_HP = 0.0;
    public static final double TOWARDS_DS = 180.0;
  }

  private final HolonomicDriveController m_controller;

  public Drive() {
    m_pigeon = new Pigeon2(DriveInfo.PIGEON_ID, RobotInfo.CANIVORE_NAME);
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());

    m_gyroOffsetDegrees = m_pigeon.getYaw().getValue();

    resetGyro();

    m_swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, DriveInfo.FRONT_LEFT_CONSTANTS),
          new SwerveModule(1, DriveInfo.FRONT_RIGHT_CONSTANTS),
          new SwerveModule(2, DriveInfo.BACK_LEFT_CONSTANTS),
          new SwerveModule(3, DriveInfo.BACK_RIGHT_CONSTANTS)
        };

    swerveOdometry =
        new SwerveDriveOdometry(
            DriveInfo.SWERVE_KINEMATICS, getGyroscopeRotation(), getPositions());

    m_controller =
        new HolonomicDriveController(
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            new ProfiledPIDController(
                5.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 7.0)));
  }

  public void driveInput(Translation2d translation, double rotation, boolean fieldRelative) {
    if (m_rotationControl == RotationControl.ClosedLoop) {
      double currentYaw = getNormalizedGyroYaw();
      double diff = m_targetRobotAngle - currentYaw;
      if (diff > 180) {
        diff -= 360;
      } else if (diff < -180) {
        diff += 360;
      }
      rotation = m_rotationController.calculate(currentYaw, currentYaw + diff);
    } else if (rotation != 0.0) {
      m_targetRobotAngle = getNormalizedGyroYaw();
    }

    ChassisSpeeds des_chassis_speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroscopeRotation())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    /*
     * TODO: Test this 254 code to handle drift in spinning translation
     * Pose2d robot_pose_vel =
     * new Pose2d(
     * des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
     * des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
     * new Rotation2d(des_chassis_speeds.omegaRadiansPerSecond *
     * Constants.kLooperDt));
     * Pose2d robot_cur_pose = new Pose2d();
     * Twist2d twist_vel = robot_cur_pose.log(robot_pose_vel);
     * ChassisSpeeds updated_chassis_speeds =
     * new ChassisSpeeds(
     * twist_vel.dx / Constants.kLooperDt,
     * twist_vel.dy / Constants.kLooperDt,
     * twist_vel.dtheta / Constants.kLooperDt);
     *
     */
    SwerveModuleState[] swerveModuleStates =
        DriveInfo.SWERVE_KINEMATICS.toSwerveModuleStates(des_chassis_speeds);
    // SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(updated_chassis_speeds);

    setModuleStates(swerveModuleStates);
  }

  public void driveInput(State state, Rotation2d rotation) {
    var desiredStates = m_controller.calculate(getPose(), state, rotation);
    setModuleStates(DriveInfo.SWERVE_KINEMATICS.toSwerveModuleStates(desiredStates));
  }

  public double getGyroYaw() {
    return m_pigeon.getYaw().getValue() - m_gyroOffsetDegrees;
  }

  public double getNormalizedGyroYaw() {
    double rawYaw = getGyroYaw();
    double normalizedYaw = Math.IEEEremainder(rawYaw, 360.0);
    if (normalizedYaw < 0) {
      normalizedYaw += 360.0;
    }
    return normalizedYaw;
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(getGyroYaw());
  }

  public void resetGyro() {
    // TODO: do a better reset here.
    m_gyroOffsetDegrees = m_pigeon.getYaw().getValue();
  }

  /* Used by Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
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

  public void update() {
    swerveOdometry.update(getGyroscopeRotation(), getPositions());

    GreyDashClient.setGyroAngle(getGyroYaw());

    SmartDashboard.putNumber("Drive Yaw", getNormalizedGyroYaw());

    for (SwerveModule mod : m_swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Raw", mod.getAngleRaw());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  public void reset() {
    resetGyro();
    resetOdometry(new Pose2d());
  }
}

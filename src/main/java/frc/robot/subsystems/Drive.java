package frc.robot.subsystems;

import frc.robot.greydash.GreyDashClient;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.Subsystem;
import frc.robot.subsystems.swerve.SwerveModule;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
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
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Drive implements Subsystem {
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] m_swerveModules;
  private ChassisSpeeds m_currentChassisSpeeds;

  private final Pigeon2 m_pigeon;
  private double m_gyroOffsetDegrees;

  @Setter private double m_targetRobotAngle = 0.0;

  @Setter private RotationControl m_rotationControl = RotationControl.OpenLoop;

  private PIDController m_rotationController = new PIDController(0.11, 0.0, 0.003);
  private PIDController m_balancePitchController = new PIDController(0.045, 0.0, 0.007);
  private PIDController m_balanceRollController = new PIDController(0.045, 0.0, 0.007);

  public enum RotationControl {
    OpenLoop,
    ClosedLoop,
  }

  public static class AnglePresets {
    public static final double TOWARDS_HP = 0.0;
    public static final double TOWARDS_DS = 180.0;
    public static final double TOWARDS_MHP_RED = -90.0;
    public static final double TOWARDS_MHP_BLUE = 90.0;
    public static final double TOWARDS_WS_BLUE = 45.0;
    public static final double TOWARDS_WSR_BLUE = 135.0;
    public static final double TOWARDS_WS_RED = -45.0;
    public static final double TOWARDS_WSR_RED = -135.0;
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

    m_currentChassisSpeeds = new ChassisSpeeds();

    swerveOdometry =
        new SwerveDriveOdometry(
            DriveInfo.SWERVE_KINEMATICS, getGyroscopeRotation(), getPositions());

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

  public void balanceDrivePitch() {
    double pitch = getGyroPitch();
    if (Math.abs(pitch) > 5.0) {
      double pitchCorrection = m_balancePitchController.calculate(pitch, 0.0);
      if (Math.abs(getNormalizedGyroYaw() - 180) < 30.0) {
        pitchCorrection *= -1.0;
      }
      driveInput(new Translation2d(pitchCorrection, 0.0), 0.0, true);
    } else {
      m_currentChassisSpeeds = new ChassisSpeeds();
      for (SwerveModule swerveModule : m_swerveModules) {
        swerveModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(90)), true);
      }
    }
  }

  public void balanceDriveRoll() {
    double roll = getGyroRoll();
    if (Math.abs(roll) > 5.0) {
      double rollCorrection = m_balanceRollController.calculate(roll, 0.0);
      if (Math.abs(getNormalizedGyroYaw() - 270) < 30.0) {
        rollCorrection *= -1.0;
      }
      driveInput(new Translation2d(rollCorrection, 0.0), 0.0, true);
    } else {
      m_currentChassisSpeeds = new ChassisSpeeds();
      for (SwerveModule swerveModule : m_swerveModules) {
        swerveModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)), true);
      }
    }
  }

  public void driveInput(Translation2d translation, double rotationVal, boolean fieldRelative) {
    double rotation = rotationVal;
    if (m_rotationControl == RotationControl.ClosedLoop) {
      double diff = m_targetRobotAngle - getNormalizedGyroYaw();
      if (diff > 180) {
        diff -= 360;
      } else if (diff < -180) {
        diff += 360;
      }

      rotation =
          m_rotationController.calculate(getNormalizedGyroYaw(), getNormalizedGyroYaw() + diff);
    } else if (rotation != 0.0) {
      m_targetRobotAngle = getNormalizedGyroYaw();
    }

    m_currentChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroscopeRotation())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
  }

  public void driveInput(State state, Rotation2d rotation) {
    m_currentChassisSpeeds = m_controller.calculate(getPose(), state, rotation);
  }

  public double getGyroPitch() {
    return m_pigeon.getPitch().getValue();
  }

  public double getGyroRoll() {
    return m_pigeon.getRoll().getValue();
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
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_gyroOffsetDegrees += pose.getRotation().getDegrees();
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
    GreyDashClient.setGyroAngle(getGyroscopeRotation().getDegrees());
  }

  public void debugDashboardUpdate() {
    double states[] = new double[8];
    int index = 0;

    for (SwerveModule mod : m_swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Raw", mod.getAngleRaw());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      states[index] = mod.getState().angle.getDegrees();
      states[index + 1] = mod.getState().speedMetersPerSecond;
      index += 2;
    }
    SmartDashboard.putNumberArray("swerve/actual", states);
    SmartDashboard.putNumber("pitch", m_pigeon.getPitch().getValue());
    SmartDashboard.putNumber("roll", m_pigeon.getRoll().getValue());
    SmartDashboard.putNumberArray(
        "swerve/odometry",
        ImmutableList.of(
                getPose().getTranslation().getX(),
                getPose().getTranslation().getY(),
                getPose().getRotation().getDegrees())
            .toArray(Double[]::new));
  }

  public void update() {
    swerveOdometry.update(getGyroscopeRotation(), getPositions());

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
    resetGyro();
    m_targetRobotAngle = getGyroYaw();
    resetOdometry(new Pose2d());
  }
}

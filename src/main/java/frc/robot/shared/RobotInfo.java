package frc.robot.shared;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Robot info, specs, dimensions. */
public final class RobotInfo {
  public static final String CANIVORE_NAME = "Canivore";

  public static final int COMPRESSOR_ID = 0;

  public static class IntakeInfo {
    public static final int FX_ID = 14;
  }

  public static class ClawInfo {
    public static final int WRIST_FX_ID = 19;
    public static final int INTAKE_FX_ID = 15;
    public static final int WRIST_HALL_ID = 5;
    public static final int CONE_SENSOR_ID = 4;
    public static final double GEAR_RATIO = (1.0 / 4.0) * (1.0 / 4.0) * (16.0 / 42.0);
  }

  public static class CANdleInfo {
    public static final int ID = 18;
  }

  public static class ElevatorInfo {
    public static final int BOTTOM_HALL_SENSOR_ID = 2;
    public static final int TOP_HALL_SENSOR_ID = 3;
    public static final int FX_ID = 16;
    public static final int FOLLOWER_FX_ID = 17;
  }

  public static class DriveInfo {
    public static final int PIGEON_ID = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 9;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 324.31;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 7;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 304.189;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 262.79;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 27;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 26;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 28;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 80.94;

    public static final double DRIVE_GEAR_RATIO =
        1 / ((12.0 / 42.0) * (28.0 / 18.0) * (15.0 / 45.0)); // 6.75:1
    public static final double ANGLE_GEAR_RATIO = 1 / ((8.0 / 24.0) * (14.0 / 72.0)); // 15.43:1

    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    /**
     * The left-to-right distance between the drivetrain wheels Should be measured from center to
     * center.
     */
    public static final double TRACKWIDTH_METERS = 0.53975;
    /**
     * The front-to-back distance between the drivetrain wheels. Should be measured from center to
     * center.
     */
    public static final double WHEELBASE_METERS = 0.53975;

    public static final double OPENLOOP_RAMP = 0.0;
    public static final double CLOSEDLOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 0.3;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /* Drive Motor Characterization Values */
    // divide by 12 to convert from volts to percent output for CTRE
    public static final double driveKS = (0.0 / 12);
    public static final double driveKV = (0.0 / 12);
    public static final double driveKA = (0.0 / 12);

    /* Motor Inverts */
    public static final boolean DRIVE_MOTOR_INVERT = true;
    public static final boolean ANGLE_MOTOR_INVERT = true;

    /* Angle Encoder Invert */
    public static final boolean CANCODER_INVERT = false;

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        8000.0 / 60.0 / DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 11.5;

    public static final SwerveModuleConfig FRONT_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET);
    public static final SwerveModuleConfig FRONT_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);
    public static final SwerveModuleConfig BACK_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);
    public static final SwerveModuleConfig BACK_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));
  }
  ;
}

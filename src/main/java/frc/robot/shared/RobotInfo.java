package frc.robot.shared;

import static frc.robot.shared.Constants.*;

/** Robot info, specs, dimensions. */
public final class RobotInfo {
  public static final String CANIVORE_NAME = "Canivore";

  public static final int ARM_FX_ID = 0;
  public static final int ARM_SOLENOID_ID = 99;

  public static final int ELEVATOR_FX_ID = 1;
  public static final int ELEVATOR_FOLLOWER_FX_ID = 2;
  public static final int ELEVATOR_BOTTOM_HALL_SENSOR_ID = 3;

  public static final int COMPRESSOR_ID = 4;

  /** Standard TalonFX RPM (multiply by Ticks/100ms). */
  public static final double TALON_FX_VELOCITY_RPM =
      (MSEC_PER_SEC * SEC_PER_MIN) / (TALON_FX_TICKS_PER_REV * TALON_FX_VELOCITY_UNIT_MS);

  public static final double WRIST_GEAR_RATIO = (1.0 / 5.0) * (1.0 / 5.0) * (16.0 / 44.0);
  /** Degrees from horizontal */
  public static final double WRIST_STOW_PRESET = 90.0;
  /** Degrees from horizontal */
  public static final double WRIST_CONE_INTAKE_PRESET = 45.0;
  /** Degrees from horizontal */
  public static final double WRIST_CUBE_INTAKE_PRESET = -8.8;
  /** Degrees from horizontal */
  public static final double WRIST_MID_PRESET = 28.0;
  /** Degrees from horizontal */
  public static final double WRIST_HIGH_PRESET = 10.0;
}

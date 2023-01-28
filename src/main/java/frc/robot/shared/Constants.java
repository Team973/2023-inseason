package frc.robot.shared;

/** A collection of constants. */
public class Constants {
  /** Integrate the output of helpers/PID. */
  public static final int PID_SPEED_CTRL = 0x00000001;

  /** ft/m. */
  public static final double FEET_PER_METER = 3.280839895;
  /** in/m. */
  public static final double INCHES_PER_METER = FEET_PER_METER * 12;
  /** m/ft. */
  public static final double METERS_PER_FOOT = 1.0 / FEET_PER_METER;
  /** m/in. */
  public static final double METERS_PER_INCH = 1.0 / INCHES_PER_METER;
  /** Gravity constant meter/sq(sec). */
  public static final double GRAVITY_CONSTANT = 9.80665;
  /** Gravity constant in/sq(sec). */
  public static final double GRAVITY_CONSTANT_INCHES = GRAVITY_CONSTANT * FEET_PER_METER * 12.0;
  /** Seconds/100msec. */
  public static final double SEC_PER_100MS = 0.1;
  /** Microseconds/millisecond. */
  public static final double USEC_PER_MSEC = 1000.0;
  /** Milliseconds/microseconds. */
  public static final double MSEC_PER_USEC = 1.0 / USEC_PER_MSEC;
  /** Milliseconds/seconds. */
  public static final double MSEC_PER_SEC = 1000.0;
  /** Seconds/milliseconds. */
  public static final double SEC_PER_MSEC = 1.0 / MSEC_PER_SEC;
  /** Microseconds/second. */
  public static final double USEC_PER_SEC = USEC_PER_MSEC * MSEC_PER_SEC;
  /** Microseconds/milliseconds. */
  public static final double SEC_PER_USEC = 1.0 / USEC_PER_SEC;
  /** Minutes/seconds. */
  public static final double MIN_PER_SEC = 1.0 / 60.0;
  /** Seconds/minute. */
  public static final double SEC_PER_MIN = 60.0;
  /** Radians/degrees. */
  public static final double RAD_PER_DEG = 2 * Math.PI / 360.0;
  /** Degrees/Radians. */
  public static final double DEG_PER_RAD = 1.0 / RAD_PER_DEG;

  /** Encoder ticks/revolution of TalonFX motor. */
  public static final double TALON_FX_TICKS_PER_REV = 2048.0;
  /** Ticks/100ms. */
  public static final double TALON_FX_VELOCITY_UNIT_MS = 100.0;
}

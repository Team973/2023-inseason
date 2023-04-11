package frc.robot.shared;

import edu.wpi.first.wpilibj.RobotController;

public class Conversions {
  public static class Time {
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

    /**
     * Get the current time in microseconds.
     *
     * @return The current time.
     */
    public static long getUsecTime() {
      return RobotController.getFPGATime();
    }

    /**
     * Get the current time in milliseconds.
     *
     * @return The current time.
     */
    public static double getMsecTime() {
      return getUsecTime() * Time.MSEC_PER_USEC;
    }

    /**
     * Get the current time in seconds.
     *
     * @return The current time.
     */
    public static double getSecTime() {
      return getUsecTime() * Time.SEC_PER_USEC;
    }
  }

  public static class Distance {
    /** ft/m. */
    public static final double FEET_PER_METER = 3.280839895;
    /** in/m. */
    public static final double INCHES_PER_METER = FEET_PER_METER * 12;
    /** m/ft. */
    public static final double METERS_PER_FOOT = 1.0 / FEET_PER_METER;
    /** m/in. */
    public static final double METERS_PER_INCH = 1.0 / INCHES_PER_METER;
  }

  public static class MathHelpers {

    /**
     * Square the value while retaining the original sign.
     *
     * @param The value to be squared.
     * @return The value squared with the orignal sign.
     */
    public static double signSquare(double value) {
      return Math.pow(value, 2) * Math.signum(value);
    }
  }
}

package frc.robot.shared;

import edu.wpi.first.wpilibj.RobotController;

/** A collection of utility functions. */
public final class Util {
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
    return getUsecTime() * Constants.MSEC_PER_USEC;
  }

  /**
   * Get the current time in seconds.
   *
   * @return The current time.
   */
  public static double getSecTime() {
    return getUsecTime() * Constants.SEC_PER_USEC;
  }

  /**
   * Get the sensor velocity in units/100ms from the a velocity that is in units/sec.
   *
   * @param sensorUnitsPerSec The velocity in units/sec.
   * @return The velocity in units/100ms.
   */
  public static double getSensorVelocity(double sensorUnitsPerSec) {
    return sensorUnitsPerSec * Constants.SEC_PER_100MS;
  }
}

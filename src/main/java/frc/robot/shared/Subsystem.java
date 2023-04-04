package frc.robot.shared;

/** Base interface for all subsystems */
public interface Subsystem {

  /**
   * Update NetworkTables values for the dashboard. Call this periodically when the robot is
   * enabled. This isn't called when connected to FMS.
   */
  public void debugDashboardUpdate();

  /**
   * Update NetworkTables values for the dashboard. Call this periodically when the robot is
   * enabled.
   */
  public void dashboardUpdate();

  /** Update the subsystem. Call this periodically when the robot is enabled. */
  public void update();

  /** Reset the subsystem. */
  public void reset();
}

package frc.robot.shared;

/** Base interface for all subsystems */
public interface Subsystem {
  /** Update the subsystem. Call this periodically when the robot is enabled. */
  public void update();

  /** Reset the subsystem. */
  public void reset();
}

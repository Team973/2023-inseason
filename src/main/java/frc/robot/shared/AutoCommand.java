package frc.robot.shared;

/** Abstract Class for auto commands. */
public abstract class AutoCommand {
  protected double m_targetMsec;
  protected double m_startMsec;

  /**
   * Checks if the target time (safety timeout) has elapsed.
   *
   * @return True if the target time has passed
   */
  public boolean hasElapsed() {
    return Util.getMsecTime() - m_startMsec >= m_targetMsec;
  }

  /** Initialize the auto command. */
  public abstract void init();

  /** Run the auto command. */
  public abstract void run();

  /**
   * Check if the auto command is completed.
   *
   * @return True if the command is completed.
   */
  public abstract boolean isCompleted();

  /**
   * Sets the target time (safety timeout) for the command.
   *
   * @param target The target time for the command.
   */
  public void setTargetMsec(double target) {
    m_targetMsec = target;
    m_startMsec = Util.getMsecTime();
  }
}

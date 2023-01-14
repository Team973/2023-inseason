package frc.robot.shared;

/** Inteface for auto commands. */
public abstract class AutoCommand {
  protected double m_targetMsec;
  protected double m_startMsec;

  public boolean hasElapsed() {
    return Util.getMsecTime() - m_startMsec >= m_targetMsec;
  }

  /** Initialize the auto command. */
  public abstract void init();

  /** Run the auto command. */
  public abstract void run();

  /** Check if the auto command is completed. */
  public abstract boolean isCompleted();

  public void setTargetMsec(double target) {
    m_targetMsec = target;
    m_startMsec = Util.getMsecTime();
  }
}

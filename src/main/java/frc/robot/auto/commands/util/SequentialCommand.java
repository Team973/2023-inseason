package frc.robot.auto.commands.util;

import frc.robot.shared.AutoCommand;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SequentialCommand extends AutoCommand {
  private final ImmutableList<AutoCommand> m_cmdList;
  private boolean m_cmdNeedsInit = true;
  private int m_currentIndex = 0;
  private Double m_timeout = Double.MAX_VALUE;

  /**
   * Constructor for sequential command class.
   *
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public SequentialCommand(AutoCommand... commands) {
    this.m_cmdList = ImmutableList.copyOf(commands);
  }

  /**
   * Constructor for sequential command class.
   *
   * @param timeout This sets the timeout for the commands.
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public SequentialCommand(double timeout, AutoCommand... commands) {
    this(commands);
    this.m_timeout = timeout;
  }

  public void init() {
    if (m_timeout != null) {
      setTargetMsec(m_timeout);
    }
  }

  public void run() {
    SmartDashboard.putNumber("sequential index", m_currentIndex);
    if (isCompleted()) {
      return;
    }

    final AutoCommand currentCommand = m_cmdList.get(m_currentIndex);

    if (m_cmdNeedsInit) {
      currentCommand.init();
      m_cmdNeedsInit = false;
    }

    currentCommand.run();

    if (currentCommand.isCompleted()) {
      m_currentIndex++;
      m_cmdNeedsInit = true;
      currentCommand.postComplete(false);
    } else if (currentCommand.hasElapsed()) {
      m_currentIndex++;
      m_cmdNeedsInit = true;
      currentCommand.postComplete(true);
    }
  }

  public boolean isCompleted() {
    return m_currentIndex >= m_cmdList.size();
  }

  public void postComplete(boolean interrupted) {}
}

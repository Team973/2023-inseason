package frc.robot.auto.commands.util;

import java.util.HashSet;

import frc.robot.shared.AutoCommand;

import com.google.common.collect.ImmutableList;

public class ConcurrentCommand extends AutoCommand {
  private final ImmutableList<AutoCommand> m_cmdList;
  private HashSet<AutoCommand> m_unfinishedCmds;
  private Double m_timeout = null;

  /**
   * Constructor for concurrent command class.
   *
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public ConcurrentCommand(AutoCommand... commands) {
    this.m_cmdList = ImmutableList.copyOf(commands);
    m_unfinishedCmds = new HashSet<>();

    m_unfinishedCmds.addAll(m_cmdList);
  }

  /**
   * Constructor for concurrent command class with timeout parameter.
   *
   * @param timeout This sets the timeout for the commands.
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public ConcurrentCommand(double timeout, AutoCommand... commands) {
    this(commands);
    this.m_timeout = timeout;
  }

  public void init() {
    if (m_timeout != null) {
      setTargetMsec(m_timeout);
    }

    for (AutoCommand command : m_cmdList) {
      command.init();
    }
  }

  public void run() {
    if (isCompleted()) {
      return;
    }

    for (AutoCommand command : m_unfinishedCmds) {
      if (!command.isCompleted()) {
        command.run();
      } else {
        m_unfinishedCmds.remove(command);
      }
    }
  }

  public boolean isCompleted() {
    for (AutoCommand command : m_unfinishedCmds) {
      if (!command.isCompleted()) {
        return false;
      } else {
        m_unfinishedCmds.remove(command);
      }
    }

    return true;
  }
}

package frc.robot.auto.commands.util;

import java.util.HashSet;

import frc.robot.shared.AutoCommand;

import com.google.common.collect.ImmutableList;

public class ConcurrentCommand extends AutoCommand {
  private final ImmutableList<AutoCommand> m_cmdList;
  private HashSet<AutoCommand> m_finishedCmds;
  private Double m_timeout = null;

  private boolean m_cmdsNeedInit = true;

  /**
   * Constructor for concurrent command class.
   *
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public ConcurrentCommand(AutoCommand... commands) {
    this.m_cmdList = ImmutableList.copyOf(commands);
    m_finishedCmds = new HashSet<>();
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

    for (var command : m_cmdList) {
      command.init();
    }
  }

  public void run() {
    if (isCompleted()) {
      return;
    }

    for (var command : m_cmdList) {
      if (m_finishedCmds.contains(command)) {
        continue;
      }

      command.run();

      if (command.isCompleted()) {
        command.postComplete(false);
        m_finishedCmds.add(command);
      } else if (command.hasElapsed()) {
        command.postComplete(true);
        m_finishedCmds.add(command);
      }
    }
  }

  public boolean isCompleted() {
    return m_finishedCmds.size() == m_cmdList.size();
  }

  public void postComplete(boolean interrupted) {
    if (interrupted) {
      for (var command : m_cmdList) {
        if (!m_finishedCmds.contains(command)) {
          command.postComplete(true);
        }
      }
    }
  }
}

package frc.robot.auto.commands.util;

import java.util.HashSet;
import java.util.Iterator;

import frc.robot.shared.AutoCommand;

import com.google.common.collect.ImmutableList;

public class ConcurrentCommand extends AutoCommand {
  private final ImmutableList<AutoCommand> m_cmdList;
  private HashSet<AutoCommand> m_unfinishedCmds;
  private Double m_timeout = null;

  private boolean m_cmdsNeedInit = true;

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
  }

  public void run() {
    if (isCompleted()) {
      return;
    }

    Iterator<AutoCommand> iterator = m_unfinishedCmds.iterator();
    while (iterator.hasNext()) {
      AutoCommand command = iterator.next();
      if (m_cmdsNeedInit) {
        command.init();
      }

      command.run();

      if (command.isCompleted()) {
        command.postComplete();
        iterator.remove();
      }
    }
    m_cmdsNeedInit = false;
  }

  public boolean isCompleted() {
    return m_unfinishedCmds.size() == 0 || hasElapsed();
  }

  public void postComplete() {}
}

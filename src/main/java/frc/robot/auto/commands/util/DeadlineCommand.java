package frc.robot.auto.commands.util;

import java.util.HashSet;
import java.util.Iterator;

import frc.robot.shared.AutoCommand;

import com.google.common.collect.ImmutableList;

public class DeadlineCommand extends AutoCommand {
  private final AutoCommand m_deadline;
  private final ImmutableList<AutoCommand> m_cmdList;
  private HashSet<AutoCommand> m_unfinishedCmds;
  private Double m_timeout = null;

  private boolean m_cmdsNeedInit = true;

  /**
   * Constructor for deadline command class.
   * This command will run until the deadline command is completed.
   *
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public DeadlineCommand(AutoCommand deadline, AutoCommand... commands) {
    m_deadline = deadline;
    m_cmdList = ImmutableList.copyOf(commands);
    m_unfinishedCmds = new HashSet<>();

    m_unfinishedCmds.addAll(m_cmdList);
  }

  /**
   * Constructor for deadline command class with timeout parameter.
   * This command will run until the timeout is reached or the deadline command is
   * completed.
   *
   * @param timeout  This sets the timeout for the commands.
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public DeadlineCommand(double timeout, AutoCommand deadline, AutoCommand... commands) {
    this(deadline, commands);
    m_timeout = timeout;
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
        command.postComplete(false);
        iterator.remove();
      } else if (command.hasElapsed()) {
        command.postComplete(true);
        iterator.remove();
      }
    }
    m_cmdsNeedInit = false;
  }

  public boolean isCompleted() {
    return m_deadline.isCompleted() || m_unfinishedCmds.size() == 0;
  }

  public void postComplete(boolean interrupted) {
  }
}

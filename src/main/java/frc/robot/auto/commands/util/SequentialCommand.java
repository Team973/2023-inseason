package frc.robot.auto.commands.util;

import frc.robot.shared.AutoCommand;

import com.google.common.collect.ImmutableList;

public class SequentialCommand extends AutoCommand {
    private final ImmutableList<AutoCommand> m_cmdList;
    private boolean m_cmdNeedsInit = true;
    private int m_currentIndex = 0;
    private Double m_timeout = null;

    public SequentialCommand(AutoCommand... commands) {
        this.m_cmdList = ImmutableList.copyOf(commands);
    }

    public SequentialCommand(double timeout, AutoCommand... commands) {
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

        final AutoCommand currentCommand = m_cmdList.get(m_currentIndex);

        if (m_cmdNeedsInit) {
            m_currentIndex++;
            m_cmdNeedsInit = true;
        }

        currentCommand.run();
    }

    public boolean isCompleted() {
        return m_currentIndex > m_cmdList.size();
    }
}

package frc.robot.auto;

import frc.robot.shared.AutoCommand;

public class AutoMode {
    public void concurrentCommand(AutoCommand... commands) {
        for (AutoCommand command : commands) {
            command.run();
        }
    }

    public void sequentialCommand(AutoCommand... commands) {
        for (AutoCommand command : commands) {
            command.init();
            command.run();

            while (!command.isCompleted()) {
                if (command.isCompleted()) {
                    break;
                }
            }
        }
    }
}

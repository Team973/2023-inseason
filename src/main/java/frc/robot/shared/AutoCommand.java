package frc.robot.shared;

/** Inteface for auto commands. */
public interface AutoCommand {
    /** Initialize the auto command. */
    public void init();

    /** Run the auto command. */
    public void run();

    /** Check if the auto command is completed. */
    public boolean isCompleted();
}

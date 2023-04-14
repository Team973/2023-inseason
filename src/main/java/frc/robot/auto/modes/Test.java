package frc.robot.auto.modes;

import frc.robot.auto.commands.TossCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.subsystems.Superstructure;

public class Test extends SequentialCommand {
  public Test(Superstructure superstructure) {
    super(new TossCommand(superstructure, 1000));
  }
}

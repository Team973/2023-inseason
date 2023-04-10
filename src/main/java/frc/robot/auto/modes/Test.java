package frc.robot.auto.modes;

import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

public class Test extends SequentialCommand {
  public Test(Superstructure superstructure) {
    super(new ScorePreloadCommand(GamePiece.Cone, GlobalState.ScoreHigh, superstructure));
  }
}

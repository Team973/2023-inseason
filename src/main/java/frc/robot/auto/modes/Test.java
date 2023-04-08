package frc.robot.auto.modes;

import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class Test extends SequentialCommand {
  public Test(Claw claw, Wrist wrist, Elevator elevator) {
    super(
        new ScorePreloadCommand(
            GamePiece.Cone, Elevator.Preset.High, WristPreset.High, claw, wrist, elevator));
  }
}

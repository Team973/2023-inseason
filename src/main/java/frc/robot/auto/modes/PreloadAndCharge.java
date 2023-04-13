package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

public class PreloadAndCharge extends SequentialCommand {

  public PreloadAndCharge(Drive drive, Superstructure superstructure) {
    super(
        // Score preload
        new ScorePreloadCommand(GamePiece.Cone, GlobalState.ScoreHigh, superstructure),

        // Go balance
        new PathPlannerTrajectoryCommand(
            drive, TrajectoryManager.PreloadAndCharge.getPathSegment(0)),
        new BalanceCommand(drive, 5000));
  }
}

package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

public class CenterPreloadAndCharge extends SequentialCommand {
  public CenterPreloadAndCharge(Drive drive, Superstructure superstructure) {
    super(
        // Score preload
        new ScorePreloadCommand(GamePiece.Cone, GlobalState.ScoreHigh, superstructure),

        // Go balance
        new PathPlannerTrajectoryCommand(
            drive, TrajectoryManager.CenterPreloadAndCharge.getPathSegment(0)),
        new WaitCommand(1000),
        new PathPlannerTrajectoryCommand(
            drive, false, TrajectoryManager.CenterPreloadAndCharge.getPathSegment(1)),
        new BalanceCommand(drive, 5000));
  }
}

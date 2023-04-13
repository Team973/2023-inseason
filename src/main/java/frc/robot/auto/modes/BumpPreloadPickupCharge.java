package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.SetCurrentGamePieceCommand;
import frc.robot.auto.commands.SuperstructureGlobalStateCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

public class BumpPreloadPickupCharge extends SequentialCommand {
  public BumpPreloadPickupCharge(Drive drive, Superstructure superstructure) {
    super(
        new ScorePreloadCommand(GamePiece.Cone, GlobalState.ScoreHigh, superstructure),

        // Drive to pickup
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(
                drive, TrajectoryManager.BumpPreloadPickupCharge.getPathSegment(0)),
            new SequentialCommand(
                new WaitCommand(1700),
                new SetCurrentGamePieceCommand(GamePiece.Cube),
                new ConcurrentCommand(
                    new SuperstructureGlobalStateCommand(
                        superstructure, GlobalState.LoadFloor, 4000),
                    new IntakeCommand(superstructure, IntakeState.In, true, 2000)))),

        // Balance
        new PathPlannerTrajectoryCommand(
            drive, false, TrajectoryManager.BumpPreloadPickupCharge.getPathSegment(1)),
        new BalanceCommand(drive, 5000));
  }
}

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

public class ChezyBump2Charge extends SequentialCommand {
  public ChezyBump2Charge(Drive drive, Superstructure superstructure) {
    super(
        new ScorePreloadCommand(GamePiece.Cone, GlobalState.ScoreHigh, superstructure),

        // Drive to pickup
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(
                drive, TrajectoryManager.ChezyBump2Charge.getPathSegment(0)),
            new SequentialCommand(
                new WaitCommand(1500),
                new SetCurrentGamePieceCommand(GamePiece.Cube),
                new ConcurrentCommand(
                    new SuperstructureGlobalStateCommand(
                        superstructure, GlobalState.LoadFloor, 4000),
                    new IntakeCommand(superstructure, IntakeState.In, true, 2000)))),

        // Drive to Score
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(
                drive, false, TrajectoryManager.ChezyBump2Charge.getPathSegment(1)),
            new SequentialCommand(
                new WaitCommand(3000),
                new SequentialCommand(
                    new SuperstructureGlobalStateCommand(
                        superstructure, GlobalState.ScoreHigh, 4000),
                    new IntakeCommand(superstructure, IntakeState.Out, true, 520)))),

        // Balance
        new PathPlannerTrajectoryCommand(
            drive, false, TrajectoryManager.ChezyBump2Charge.getPathSegment(2)),
        new BalanceCommand(drive, 6000));
  }
}

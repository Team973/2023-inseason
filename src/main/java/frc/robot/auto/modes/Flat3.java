package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
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

public class Flat3 extends SequentialCommand {

  public Flat3(Drive drive, Superstructure superstructure) {
    super(
        // Score preload
        new ScorePreloadCommand(GamePiece.Cone, GlobalState.ScoreMid, superstructure),

        // Drive to pickup
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(drive, TrajectoryManager.Flat3.getPathSegment(0)),
            new SequentialCommand(
                new WaitCommand(1000),
                new SetCurrentGamePieceCommand(GamePiece.Cube),
                new ConcurrentCommand(
                    new SuperstructureGlobalStateCommand(
                        superstructure, GlobalState.LoadFloor, 1000),
                    new IntakeCommand(superstructure, IntakeState.In, true, 2000)))),

        // Score cube
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(
                drive, false, TrajectoryManager.Flat3.getPathSegment(1)),
            new SequentialCommand(
                new WaitCommand(1500),
                new SuperstructureGlobalStateCommand(superstructure, GlobalState.ScoreMid, 1000))),
        new IntakeCommand(superstructure, IntakeState.Out, true, 500),

        // Pickup cone
        new ConcurrentCommand(
            new SequentialCommand(
                new WaitCommand(500),
                new SuperstructureGlobalStateCommand(superstructure, GlobalState.PostScore, 1000)),
            new SequentialCommand(
                new WaitCommand(1000),
                new SetCurrentGamePieceCommand(GamePiece.Cone),
                new ConcurrentCommand(
                    new SuperstructureGlobalStateCommand(
                        superstructure, GlobalState.LoadFloor, 1000),
                    new IntakeCommand(superstructure, IntakeState.In, false, 1000))),
            new PathPlannerTrajectoryCommand(
                drive, false, TrajectoryManager.Flat3.getPathSegment(2))),

        // Score cone
        new ConcurrentCommand(
            new SuperstructureGlobalStateCommand(superstructure, GlobalState.Stow, 1000),
            new PathPlannerTrajectoryCommand(
                drive, false, TrajectoryManager.Flat3.getPathSegment(3)),
            new SequentialCommand(
                new WaitCommand(1500),
                new SuperstructureGlobalStateCommand(superstructure, GlobalState.ScoreMid, 1000),
                new IntakeCommand(superstructure, IntakeState.Out, true, 1000))));
  }
}

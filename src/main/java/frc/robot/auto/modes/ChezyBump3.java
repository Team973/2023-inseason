package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.SetCurrentGamePieceCommand;
import frc.robot.auto.commands.SuperstructureGlobalStateCommand;
import frc.robot.auto.commands.TossCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.LambdaCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

public class ChezyBump3 extends SequentialCommand {
  public ChezyBump3(Drive drive, Superstructure superstructure) {
    super(
        new LambdaCommand(() -> drive.enableBrakeMode()),
        new ScorePreloadCommand(GamePiece.Cone, GlobalState.ScoreMid, superstructure),
        new LambdaCommand(() -> drive.disableBrakeMode()),

        // Drive to pickup/score
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(drive, TrajectoryManager.ChezyBump3.getPathSegment(0)),
            new SequentialCommand(
                new WaitCommand(1500),
                new SetCurrentGamePieceCommand(GamePiece.Cube),
                new ConcurrentCommand(
                    new SuperstructureGlobalStateCommand(
                        superstructure, GlobalState.LoadFloor, 3500),
                    new IntakeCommand(superstructure, IntakeState.In, true, 2000)),

                // Score
                new WaitCommand(2400),
                new LambdaCommand(() -> drive.enableBrakeMode()),
                new SuperstructureGlobalStateCommand(superstructure, GlobalState.ScoreMid, 4000),
                new LambdaCommand(() -> drive.disableBrakeMode()),
                new IntakeCommand(superstructure, IntakeState.Out, true, 250))),

        // drive to pick up/toss/balance
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(
                drive, false, TrajectoryManager.ChezyBump3.getPathSegment(1)),
            new SequentialCommand(
                new WaitCommand(2000),
                new SetCurrentGamePieceCommand(GamePiece.Cone),
                new ConcurrentCommand(
                    new SuperstructureGlobalStateCommand(
                        superstructure, GlobalState.LoadFloor, 4500),
                    new IntakeCommand(superstructure, IntakeState.In, true, 2000)),
                new WaitCommand(1450),
                new TossCommand(superstructure, 10000))));
  }
}

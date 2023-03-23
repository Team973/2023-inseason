package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.SetCurrentGamePieceCommand;
import frc.robot.auto.commands.WristPresetCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class PreloadPickupScoreCharge extends SequentialCommand {

  public PreloadPickupScoreCharge(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        new ScorePreloadCommand(GamePiece.Cone, claw, wrist, elevator),

        // Drive to pickup
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive,
                TrajectoryManager.getPathSegment(TrajectoryManager.PreloadPickupScoreCharge, 0)),
            new SequentialCommand(
                new WaitCommand(1000),
                new SetCurrentGamePieceCommand(GamePiece.Cube),
                new ConcurrentCommand(
                    new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 1000),
                    new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 1000),
                    new IntakeCommand(claw, IntakeState.In, 2000)))),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000)),
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(
                drive,
                false,
                TrajectoryManager.getPathSegment(TrajectoryManager.PreloadPickupScoreCharge, 1)),
            new SequentialCommand(
                new WaitCommand(1500),
                new ConcurrentCommand(
                    new ElevatorPresetCommand(elevator, Elevator.Preset.High, 1000),
                    new WristPresetCommand(wrist, WristPreset.High, 10.0, 1000)))),
        new WaitCommand(500),
        new IntakeCommand(claw, IntakeState.Out, 500),
        new WaitCommand(200),
        new SetCurrentGamePieceCommand(GamePiece.None),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000),
            new PathPlannerTrajectoryCommand(
                drive,
                false,
                TrajectoryManager.getPathSegment(TrajectoryManager.PreloadPickupScoreCharge, 2))),
        new BalanceCommand(drive, 5000));
  }
}

package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
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

public class PreloadScoreTwo extends SequentialCommand {
  public PreloadScoreTwo(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        // Score preload
        new IntakeCommand(claw, IntakeState.In, 200),
        new WristPresetCommand(wrist, WristPreset.Offset, 10.0, 500),
        new ElevatorPresetCommand(elevator, Elevator.Preset.High, 4000),
        new WristPresetCommand(wrist, WristPreset.High, 1.0, 2000),
        new IntakeCommand(claw, IntakeState.Out, 500),
        new WaitCommand(200),
        new SetCurrentGamePieceCommand(GamePiece.None),

        // Drive to pickup
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive, TrajectoryManager.getPathSegment(TrajectoryManager.PreloadScoreTwo, 0)),
            new SequentialCommand(
                new WaitCommand(2000),
                new SetCurrentGamePieceCommand(GamePiece.Cube),
                new ConcurrentCommand(
                    new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 4000),
                    new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 2000),
                    new IntakeCommand(claw, IntakeState.In, 1000)))),

        // Stow and Score
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000)),
        new PathPlannerTrajectoryCommand(
            drive, false, TrajectoryManager.getPathSegment(TrajectoryManager.PreloadScoreTwo, 1)),
        new WristPresetCommand(wrist, WristPreset.Offset, 10.0, 500),
        new ElevatorPresetCommand(elevator, Elevator.Preset.High, 4000),
        new WristPresetCommand(wrist, WristPreset.High, 10.0, 2000),
        new IntakeCommand(claw, IntakeState.Out, 500),
        new WaitCommand(200),
        new SetCurrentGamePieceCommand(GamePiece.None));

    // Drive to Pickup
    new ConcurrentCommand(
        new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
        new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
        new PathPlannerTrajectoryCommand(
            drive, false, TrajectoryManager.getPathSegment(TrajectoryManager.PreloadScoreTwo, 2)),
        new WaitCommand(200),
        new SetCurrentGamePieceCommand(GamePiece.Cone),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 4000),
            new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 2000),
            new IntakeCommand(claw, IntakeState.In, 1000)));

    // Score and Stow
    new ConcurrentCommand(
        new WaitCommand(2000),
        new ElevatorPresetCommand(elevator, Elevator.Preset.High, 4000),
        new WristPresetCommand(wrist, WristPreset.High, 10.0, 2000),
        new IntakeCommand(claw, IntakeState.Out, 500),
        new WaitCommand(200),
        new SetCurrentGamePieceCommand(GamePiece.None));
  }
}

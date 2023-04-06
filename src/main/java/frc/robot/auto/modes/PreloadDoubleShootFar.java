package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.WristPresetCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class PreloadDoubleShootFar extends SequentialCommand {

  public PreloadDoubleShootFar(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        new ScorePreloadCommand(
            GamePiece.Cone, Elevator.Preset.High, WristPreset.High, claw, wrist, elevator),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive,
                TrajectoryManager.getPathSegment(TrajectoryManager.PreloadDoubleShootFar, 0))),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 1000),
            new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 1000),
            new IntakeCommand(claw, IntakeState.In, 2000)),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive,
                TrajectoryManager.getPathSegment(TrajectoryManager.PreloadDoubleShootFar, 1))),
        new ConcurrentCommand(
            new WristPresetCommand(wrist, WristPreset.Floor, 10.00, 1000),
            new IntakeCommand(claw, IntakeState.Out, 1000)),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive,
                TrajectoryManager.getPathSegment(TrajectoryManager.PreloadDoubleShootFar, 2))),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 1000),
            new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 1000),
            new IntakeCommand(claw, IntakeState.In, 2000)),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive,
                TrajectoryManager.getPathSegment(TrajectoryManager.PreloadDoubleShootFar, 3)),
            new ConcurrentCommand(
                new WristPresetCommand(wrist, WristPreset.Floor, 10.00, 1000),
                new IntakeCommand(claw, IntakeState.Out, 1000))));
  }
}

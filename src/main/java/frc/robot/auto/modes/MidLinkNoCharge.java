package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.SetCurrentGamePieceCommand;
import frc.robot.auto.commands.WristPresetCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class MidLinkNoCharge extends SequentialCommand {

  public MidLinkNoCharge(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        new ScorePreloadCommand(
            GamePiece.Cone, Elevator.Preset.Mid, WristPreset.Mid, claw, wrist, elevator),

        // Drive to pickup
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive, TrajectoryManager.MidLinkNoCharge.getPathSegment(0)),
            new SequentialCommand(
                new WaitCommand(1000),
                new SetCurrentGamePieceCommand(GamePiece.Cube),
                new ConcurrentCommand(
                    new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 1000),
                    new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 1000),
                    new IntakeCommand(claw, IntakeState.In, 2000)))),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000),
            new PathPlannerTrajectoryCommand(
                drive, false, TrajectoryManager.MidLinkNoCharge.getPathSegment(1)),
            new SequentialCommand(
                new WaitCommand(1500),
                new ConcurrentCommand(
                    new ElevatorPresetCommand(elevator, Elevator.Preset.Mid, 1000),
                    new WristPresetCommand(wrist, WristPreset.Mid, 10.0, 1000)))),
        new IntakeCommand(claw, IntakeState.Out, 500),
        new ConcurrentCommand(
            new SequentialCommand(
                new WaitCommand(500),
                new ConcurrentCommand(
                    new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000),
                    new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000))),
            new ConcurrentCommand(
                new SequentialCommand(
                    new WaitCommand(1000),
                    new SetCurrentGamePieceCommand(GamePiece.Cone),
                    new ConcurrentCommand(
                        new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 1000),
                        new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 1000),
                        new IntakeCommand(claw, IntakeState.In, 2000))),
                new PathPlannerTrajectoryCommand(
                    drive, false, TrajectoryManager.MidLinkNoCharge.getPathSegment(2)))),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000),
            new PathPlannerTrajectoryCommand(
                drive, false, TrajectoryManager.MidLinkNoCharge.getPathSegment(3)),
            new SequentialCommand(
                new WaitCommand(1500),
                new ConcurrentCommand(
                    new ElevatorPresetCommand(elevator, Elevator.Preset.Mid, 1000),
                    new WristPresetCommand(wrist, WristPreset.Mid, 10.0, 1000)))),
        new IntakeCommand(claw, IntakeState.Out, 500),
        new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000),
        new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000));
  }
}

package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
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

public class PreloadPickupCharge extends SequentialCommand {

  public PreloadPickupCharge(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        // Score preload
        new IntakeCommand(claw, IntakeState.In, 100),
        new WristPresetCommand(wrist, WristPreset.Offset, 10.0, 500),
        new ElevatorPresetCommand(elevator, Elevator.Presets.high, 4000),
        new WristPresetCommand(wrist, WristPreset.High, 10.0, 2000),
        new IntakeCommand(claw, IntakeState.Out, 200),
        new ConcurrentCommand(
            new WristPresetCommand(wrist, WristPreset.HighBack, 5.0, 500),
            new IntakeCommand(claw, IntakeState.Out, 200)),
        new SetCurrentGamePieceCommand(GamePiece.None),

        // Drive to pickup
        new ConcurrentCommand(
            new IntakeCommand(claw, IntakeState.Out, 500),
            new SequentialCommand(
                new WaitCommand(500),
                new ElevatorPresetCommand(elevator, Elevator.Presets.stow, 1000),
                new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000)),
            new PathPlannerTrajectoryCommand(
                drive, TrajectoryManager.getPathSegment(TrajectoryManager.PreloadPickupCharge, 0)),
            new SequentialCommand(
                new WaitCommand(2000),
                new SetCurrentGamePieceCommand(GamePiece.Cone),
                new ConcurrentCommand(
                    new ElevatorPresetCommand(elevator, Elevator.Presets.floor, 4000),
                    new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 2000),
                    new IntakeCommand(claw, IntakeState.In, 1000)))),

        // Stow
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Presets.stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 1000)),

        // Balance
        new PathPlannerTrajectoryCommand(
            drive,
            false,
            TrajectoryManager.getPathSegment(TrajectoryManager.PreloadPickupCharge, 1)),
        new BalanceCommand(drive, 5000));
  }
}

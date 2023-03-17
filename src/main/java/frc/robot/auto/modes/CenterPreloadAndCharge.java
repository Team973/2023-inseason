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

public class CenterPreloadAndCharge extends SequentialCommand {
  public CenterPreloadAndCharge(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        new IntakeCommand(claw, IntakeState.In, 100),
        new WristPresetCommand(wrist, WristPreset.Offset, 10.0, 500),
        new ElevatorPresetCommand(elevator, Elevator.Preset.High, 4000),
        new WristPresetCommand(wrist, WristPreset.High, 10.0, 2000),
        new IntakeCommand(claw, IntakeState.Out, 200),
        new ConcurrentCommand(
            new WristPresetCommand(wrist, WristPreset.HighBack, 5.0, 500),
            new IntakeCommand(claw, IntakeState.Out, 200)),
        new SetCurrentGamePieceCommand(GamePiece.None),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive,
                TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndCharge, 0))),
        new WaitCommand(1000),
        new PathPlannerTrajectoryCommand(
            drive,
            false,
            TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndCharge, 1)),
        new BalanceCommand(drive, 5000));
  }
}
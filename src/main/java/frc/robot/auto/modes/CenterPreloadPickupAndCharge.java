package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
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

public class CenterPreloadPickupAndCharge extends SequentialCommand {
  public CenterPreloadPickupAndCharge(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        new ScorePreloadCommand(
            GamePiece.Cone, Elevator.Preset.High, WristPreset.High, claw, wrist, elevator),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive,
                TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndCharge, 0))),
        new WaitCommand(1000),
        new ConcurrentCommand(
            new PathPlannerTrajectoryCommand(
                drive,
                false,
                TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndCharge, 1)),
            new ElevatorPresetCommand(elevator, Elevator.Preset.Floor, 1000),
            new WristPresetCommand(wrist, WristPreset.Floor, 10.0, 2000),
            new IntakeCommand(claw, IntakeState.In, 2000)),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000)),
        new BalanceCommand(drive, 5000));
  }
}

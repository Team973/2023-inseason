package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.ScorePreloadCommand;
import frc.robot.auto.commands.WristPresetCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class CenterPreloadAndCharge extends SequentialCommand {
  public CenterPreloadAndCharge(Drive drive, Claw claw, Elevator elevator, Wrist wrist) {
    super(
        new ScorePreloadCommand(
            GamePiece.Cone, Elevator.Preset.High, WristPreset.High, claw, wrist, elevator),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Preset.Stow, 1000),
            new WristPresetCommand(wrist, WristPreset.Stow, 10.0, 2000),
            new PathPlannerTrajectoryCommand(
                drive, TrajectoryManager.CenterPreloadAndCharge.getPathSegment(0))),
        new WaitCommand(1000),
        new PathPlannerTrajectoryCommand(
            drive, false, TrajectoryManager.CenterPreloadAndCharge.getPathSegment(1)),
        new BalanceCommand(drive, 5000));
  }
}

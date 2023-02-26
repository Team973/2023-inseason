package frc.robot.auto.modes;

import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.SetCurrentGamePieceCommand;
import frc.robot.auto.commands.WristPresetCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Claw.WristPreset;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.PathConstraints;

public class PreloadAndCharge extends SequentialCommand {
  public PreloadAndCharge(Drive drive, Claw claw, Elevator elevator) {
    super(
        new SetCurrentGamePieceCommand(GamePiece.Cone),
        new IntakeCommand(claw, IntakeState.In, 100),
        new ElevatorPresetCommand(elevator, Elevator.Presets.high, 4000),
        new WristPresetCommand(claw, WristPreset.High, 2000),
        new IntakeCommand(claw, IntakeState.Out, 200),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Presets.stow, 1000),
            new WristPresetCommand(claw, WristPreset.Stow, 2000),
            new PathPlannerTrajectoryCommand(
                drive, "PreloadAndCharge", new PathConstraints(4, 3), true)));
  }
}

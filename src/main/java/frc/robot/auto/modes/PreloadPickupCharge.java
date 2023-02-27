package frc.robot.auto.modes;

import java.util.HashMap;
import java.util.List;

import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.FollowPathWithEvents;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.SetCurrentGamePieceCommand;
import frc.robot.auto.commands.WristPresetCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.shared.AutoCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Claw.WristPreset;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PreloadPickupCharge extends SequentialCommand {
  private static HashMap<String, AutoCommand> m_eventMap = new HashMap<>();

  private static List<PathPlannerTrajectory> m_trajectories =
      PathPlanner.loadPathGroup("PreloadPickupCharge", true, new PathConstraints(4, 3));

  public PreloadPickupCharge(Drive drive, Claw claw, Elevator elevator) {
    super(
        // Score preload
        new SetCurrentGamePieceCommand(GamePiece.Cone),
        new IntakeCommand(claw, IntakeState.In, 100),
        new ElevatorPresetCommand(elevator, Elevator.Presets.high, 4000),
        new WristPresetCommand(claw, WristPreset.High, 2000),
        new IntakeCommand(claw, IntakeState.Out, 200),
        new SetCurrentGamePieceCommand(GamePiece.None),

        // Drive to pickup
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Presets.stow, 1000),
            new WristPresetCommand(claw, WristPreset.Stow, 2000),
            new FollowPathWithEvents(
                new PathPlannerTrajectoryCommand(drive, m_trajectories.get(0)),
                m_trajectories.get(0).getMarkers(),
                m_eventMap)),

        // Pickup
        new SetCurrentGamePieceCommand(GamePiece.Cube),
        new ElevatorPresetCommand(elevator, Elevator.Presets.floor, 4000),
        new WristPresetCommand(claw, WristPreset.Floor, 2000),
        new IntakeCommand(claw, IntakeState.In, 1000),

        // Stow
        new ElevatorPresetCommand(elevator, Elevator.Presets.stow, 4000),
        new WristPresetCommand(claw, WristPreset.Stow, 2000),

        // Balance
        new FollowPathWithEvents(
            new PathPlannerTrajectoryCommand(drive, m_trajectories.get(1)),
            m_trajectories.get(1).getMarkers(),
            m_eventMap),
        new BalanceCommand(drive, 5000));
  }
}

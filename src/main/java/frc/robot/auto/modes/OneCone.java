package frc.robot.auto.modes;

import frc.robot.auto.commands.DriveTrajectoryCommand;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.SetDrivePositionCommand;
import frc.robot.auto.commands.TrajectoryManager;
import frc.robot.auto.commands.WristAngleCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class OneCone extends SequentialCommand {
  public OneCone(Drive drive, Claw claw, Elevator elevator, TrajectoryManager trajectoryManager) {
    super(
        new SetDrivePositionCommand(
            drive,
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)),
            Rotation2d.fromDegrees(180.0)),
        new IntakeCommand(claw, IntakeState.In, GamePiece.Cone, 100),
        new ElevatorPresetCommand(elevator, Elevator.Presets.high, 4000),
        new WaitCommand(500),
        new WristAngleCommand(claw, Claw.ConePresets.high, 2000),
        new IntakeCommand(claw, IntakeState.Out, GamePiece.Cone, 1500),
        new WaitCommand(500),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Presets.stow, 1000),
            new WristAngleCommand(claw, Claw.ConePresets.stow, 2000)),
        new DriveTrajectoryCommand(drive, trajectoryManager.getTrajectoryA()));
  }
}

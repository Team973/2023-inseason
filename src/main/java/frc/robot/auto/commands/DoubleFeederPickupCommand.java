package frc.robot.auto.commands;

import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.AutoCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.LimelightHelpers;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

import edu.wpi.first.math.geometry.Translation2d;

public class DoubleFeederPickupCommand extends AutoCommand {
  SequentialCommand m_command;

  public DoubleFeederPickupCommand(
      Drive drive, Elevator elevator, Wrist wrist, Claw claw, GamePiece gamePiece) {
    var pose = LimelightHelpers.getTargetPose3d_CameraSpace("");

    drive.driveInput(
        new Translation2d(pose.getX(), pose.getY()), pose.getRotation().getAngle(), true);

    m_command =
        new SequentialCommand(
            new SetCurrentGamePieceCommand(gamePiece),
            new IntakeCommand(claw, IntakeState.In, 4.0),
            new WristPresetCommand(wrist, WristPreset.Offset, 10.0, 500),
            new ElevatorPresetCommand(elevator, Elevator.Preset.Hp, 4.0),
            new WristPresetCommand(wrist, Wrist.WristPreset.HP, 4.0, 4.0),
            new IntakeCommand(claw, IntakeState.In, 4.0),
            new WaitCommand(200),
            new SetCurrentGamePieceCommand(GamePiece.None));
  }

  public void init() {
    m_command.init();
  }

  public void run() {
    m_command.run();
  }

  public boolean isCompleted() {
    return m_command.isCompleted();
  }

  public void postComplete(boolean interrupted) {}
}

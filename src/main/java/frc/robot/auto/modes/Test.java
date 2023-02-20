package frc.robot.auto.modes;

import frc.robot.auto.commands.DriveTrajectoryCommand;
import frc.robot.auto.commands.SetDrivePositionCommand;
import frc.robot.auto.commands.TrajectoryManager;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Test extends SequentialCommand {
  public Test(Drive drive, TrajectoryManager trajectoryManager) {
    super(
        new SetDrivePositionCommand(
            drive,
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)),
            Rotation2d.fromDegrees(180.0)),
        new DriveTrajectoryCommand(drive, trajectoryManager.getTrajectoryA()));
  }
}
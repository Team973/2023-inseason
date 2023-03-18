package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.BalanceCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.SetDrivePositionFromLimelightCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.subsystems.Drive;

public class Test extends SequentialCommand {
  public Test(Drive drive) {
    super(
        new PathPlannerTrajectoryCommand(
            drive, TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndPickup, 0)),
        new WaitCommand(1000),
        new SetDrivePositionFromLimelightCommand(drive),
        new PathPlannerTrajectoryCommand(
            drive,
            false,
            TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndPickup, 1)),
        new BalanceCommand(drive, 5000));
  }
}

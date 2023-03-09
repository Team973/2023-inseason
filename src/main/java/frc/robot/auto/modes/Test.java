package frc.robot.auto.modes;

import frc.robot.auto.TrajectoryManager;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.subsystems.Drive;

public class Test extends SequentialCommand {
  public Test(Drive drive) {
    super(
        new PathPlannerTrajectoryCommand(
            drive, TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndPickup, 0)),
        new PathPlannerTrajectoryCommand(
            drive,
            false,
            TrajectoryManager.getPathSegment(TrajectoryManager.CenterPreloadAndPickup, 1)));
  }
}

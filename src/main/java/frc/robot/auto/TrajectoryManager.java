package frc.robot.auto;

import java.util.Arrays;
import java.util.List;

import frc.robot.Robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class TrajectoryManager {

  public static final TrajectoryPair PreloadAndCharge = new TrajectoryPair("PreloadAndCharge", true,
      new PathConstraints(4, 3));

  public static final TrajectoryPair CenterPreloadAndCharge = new TrajectoryPair("CenterPreloadAndCharge", true, true,
      new PathConstraints(4, 3));

  public static final TrajectoryPair PreloadPickupCharge = new TrajectoryPair("PreloadPickupCharge", true, false,
      new PathConstraints(4, 3));

  public static final TrajectoryPair CenterPreloadAndPickup = new TrajectoryPair("CenterPreloadAndPickup", true, false,
      new PathConstraints(1, 2));

  public static final TrajectoryPair PreloadPickupScoreCharge = new TrajectoryPair("PreloadPickupScoreCharge", true,
      false, new PathConstraints(4, 3));

  public static final TrajectoryPair MidLinkNoCharge = new TrajectoryPair("LinkMid", true, false,
      new PathConstraints(4, 3));

  public static class TrajectoryPair {

    private final List<PathPlannerTrajectory> m_trajectory;

    public TrajectoryPair(String basename, boolean reversed, PathConstraints constraints) {
      this(basename, false, reversed, constraints);
    }

    public TrajectoryPair(
        String basename,
        boolean group,
        boolean reversed,
        PathConstraints constraint,
        PathConstraints... constraints) {

      if (group) {
        m_trajectory = PathPlanner.loadPathGroup(basename + "_Group", reversed, constraint, constraints);
      } else {
        m_trajectory = Arrays.asList(PathPlanner.loadPath(basename, constraint, reversed));
      }
    }

    public PathPlannerTrajectory get(int sequenceNum) {
      return m_trajectory.get(sequenceNum);
    }

    public PathPlannerTrajectory get() {
      return get(0);
    }

    public Determinator getPathSegment(int sequenceNum) {
      return () -> PathPlannerTrajectory.transformTrajectoryForAlliance(
          this.get(sequenceNum), Robot.getCalculatedAlliance());
    }
  }

  public interface Determinator {
    PathPlannerTrajectory determine();
  }
}

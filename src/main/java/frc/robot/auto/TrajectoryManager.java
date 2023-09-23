package frc.robot.auto;

import java.util.Arrays;
import java.util.List;

import frc.robot.Robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class TrajectoryManager {

  public static final TrajectoryPair Flat1Charge =
      new TrajectoryPair("Flat1Charge", true, new PathConstraints(4, 3));

  public static final TrajectoryPair Center1Charge =
      new TrajectoryPair("Center1Charge", true, true, new PathConstraints(4, 3));

  public static final TrajectoryPair Flat1HoldCharge =
      new TrajectoryPair("Flat1HoldCharge", true, false, new PathConstraints(4, 3));

  public static final TrajectoryPair Bump1HoldCharge =
      new TrajectoryPair("Bump1HoldCharge", true, false, new PathConstraints(4, 3));

  public static final TrajectoryPair Bump2 =
      new TrajectoryPair("Bump2", true, false, new PathConstraints(4, 3));

  public static final TrajectoryPair Bump3 =
      new TrajectoryPair("Bump3", true, false, new PathConstraints(4, 3.5));

  public static final TrajectoryPair Bump3Charge =
      new TrajectoryPair("Bump3Charge", true, false, new PathConstraints(4, 3.5));

  public static final TrajectoryPair Bump2Charge =
      new TrajectoryPair("Bump2Charge", true, false, new PathConstraints(4, 3));

  public static final TrajectoryPair ChezyBump2Charge =
      new TrajectoryPair("ChezyBump2Charge", true, false, new PathConstraints(4, 3));

  public static final TrajectoryPair Center1Hold =
      new TrajectoryPair("Center1HoldCharge", true, false, new PathConstraints(1, 2));

  public static final TrajectoryPair Flat2Charge =
      new TrajectoryPair("Flat2Charge", true, false, new PathConstraints(4, 3));

  public static final TrajectoryPair Flat3 =
      new TrajectoryPair("Flat3", true, false, new PathConstraints(4, 3));

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
        m_trajectory =
            PathPlanner.loadPathGroup(basename + "_Group", reversed, constraint, constraints);
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
      return () ->
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              this.get(sequenceNum), Robot.getCalculatedAlliance());
    }
  }

  public interface Determinator {
    PathPlannerTrajectory determine();
  }
}

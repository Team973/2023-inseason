package frc.robot.auto;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import frc.robot.Robot;
import frc.robot.shared.Constants.GamePiece;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class TrajectoryManager {

  public static final TrajectoryPair PreloadAndCharge =
      new TrajectoryPair(
          "PreloadAndCharge",
          new PathConstraints(4, 3),
          true,
          Arrays.asList(GamePiece.Cone, GamePiece.Cube));

  public static final TrajectoryPair CenterPreloadAndCharge =
      new TrajectoryPair(
          "CenterPreloadAndCharge",
          true,
          new PathConstraints(4, 3),
          true,
          Arrays.asList(GamePiece.Cone));

  public static final TrajectoryPair PreloadPickupCharge =
      new TrajectoryPair(
          "PreloadPickupCharge",
          true,
          new PathConstraints(4, 3),
          false,
          Arrays.asList(GamePiece.Cone));

  public static final TrajectoryPair DriveBack =
      new TrajectoryPair(
          "DriveBack", false, new PathConstraints(4, 3), true, Arrays.asList(GamePiece.Cone));

  public static final TrajectoryPair CenterPreloadAndPickup =
      new TrajectoryPair(
          "CenterPreloadAndPickup",
          true,
          new PathConstraints(4, 3),
          true,
          Arrays.asList(GamePiece.Cone));

  public static class TrajectoryPair {

    private final HashMap<GamePiece, List<PathPlannerTrajectory>> gamePieceMapping;
    private final Set<GamePiece> availableGamePieces;

    public TrajectoryPair(
        String basename,
        PathConstraints constraints,
        boolean reversed,
        List<GamePiece> availableGamePieces) {
      this(basename, false, constraints, reversed, availableGamePieces);
    }

    public TrajectoryPair(
        String basename,
        boolean group,
        PathConstraints constraints,
        boolean reversed,
        List<GamePiece> availableGamePieces) {
      gamePieceMapping = new HashMap<>();

      this.availableGamePieces = new HashSet<>(availableGamePieces);

      if (group) {
        for (var p : availableGamePieces) {
          gamePieceMapping.put(
              p,
              PathPlanner.loadPathGroup(
                  p.toString() + "_" + basename + "_Group", reversed, constraints));
        }

      } else {
        for (var p : availableGamePieces) {
          gamePieceMapping.put(
              p,
              Arrays.asList(
                  PathPlanner.loadPath(p.toString() + "_" + basename, constraints, reversed)));
        }
      }
    }

    public PathPlannerTrajectory get(GamePiece p, int sequenceNum) {
      return gamePieceMapping.get(p).get(sequenceNum);
    }

    public PathPlannerTrajectory get(GamePiece p) {
      return gamePieceMapping.get(p).get(0);
    }
  }

  public interface Determinator {
    PathPlannerTrajectory determine();
  }

  public static Determinator getPath(TrajectoryPair p) {
    return getPathSegment(p, 0);
  }

  public static Determinator getPathSegment(TrajectoryPair p, int sequenceNum) {
    return () ->
        p.availableGamePieces.contains(Robot.getPreloadGamePiece())
            ? PathPlannerTrajectory.transformTrajectoryForAlliance(
                p.get(Robot.getPreloadGamePiece(), sequenceNum), Robot.getCalculatedAlliance())
            : null;
  }
}

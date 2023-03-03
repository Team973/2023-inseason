package frc.robot.auto;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

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

  public static class TrajectoryPair {

    private final HashMap<GamePiece, PathPlannerTrajectory> gamePieceMapping;

    public TrajectoryPair(
        String basename,
        PathConstraints constraints,
        boolean reversed,
        List<GamePiece> availableGamePieces) {
      gamePieceMapping = new HashMap<>();

      for (var p : availableGamePieces) {
        gamePieceMapping.put(
            p, PathPlanner.loadPath(p.toString() + "_" + basename, constraints, reversed));
      }
    }

    public PathPlannerTrajectory get(GamePiece p) {
      return gamePieceMapping.get(p);
    }
  }

  public interface Determinator {
    PathPlannerTrajectory determine();
  }

  public static Determinator buildPath(TrajectoryPair p) {
    return () ->
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            p.get(Robot.getPreloadGamePiece()), Robot.getCalculatedAlliance());
  }
}

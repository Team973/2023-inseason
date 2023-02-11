package frc.robot.auto.commands;

import java.util.Arrays;
import java.util.List;

import frc.robot.shared.RobotInfo.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class TrajectoryManager {
  public final TrajectoryConfig m_config =
      new TrajectoryConfig(DriveInfo.MAX_VELOCITY_METERS_PER_SECOND, 3.0);

  @Getter private final Trajectory m_trajectoryA;
  @Getter private final Trajectory m_trajectoryB;
  @Getter private final Trajectory m_trajectoryC;

  public TrajectoryManager() {
    // Trajectory A
    m_trajectoryA = createTrajectoryA();
    // Trajectory B
    m_trajectoryB = createTrajectoryB();
    // Trajectory C
    m_trajectoryC = createTrajectoryC();
  }

  private Trajectory createTrajectoryA() {
    Pose2d start = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    List<Translation2d> waypoints =
        Arrays.asList(new Translation2d(3.35, 0.0), new Translation2d(4.57, 1.52));
    Pose2d end = new Pose2d(new Translation2d(2.44, 1.52), Rotation2d.fromDegrees(0.0));
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
  }

  private Trajectory createTrajectoryB() {
    Pose2d start = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    List<Translation2d> waypoints =
        Arrays.asList(
            new Translation2d(1.0, 1.0), new Translation2d(2.0, 0.0), new Translation2d(1.0, -1.0));
    Pose2d end = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
  }

  private Trajectory createTrajectoryC() {
    Pose2d start = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    List<Translation2d> waypoints =
        Arrays.asList(
            new Translation2d(1.0, 1.0), new Translation2d(2.0, 0.0), new Translation2d(1.0, -1.0));
    Pose2d end = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
  }
}

package frc.robot.auto.commands;

import java.util.*;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Timer;

public class FollowPathWithEvents extends AutoCommand {
  private final AutoCommand pathFollowingCommand;
  private final List<PathPlannerTrajectory.EventMarker> pathMarkers;
  private final Map<String, AutoCommand> eventMap;

  private final Map<AutoCommand, Boolean> currentCommands = new HashMap<>();
  private final List<PathPlannerTrajectory.EventMarker> unpassedMarkers = new ArrayList<>();
  private final Timer timer = new Timer();
  private boolean isCompleted = true;

  public FollowPathWithEvents(
      Drive drive,
      String pathFile,
      PathConstraints constraints,
      boolean reversed,
      Map<String, AutoCommand> eventMap) {
    PathPlannerTrajectory path = PathPlanner.loadPath(pathFile, constraints, reversed);
    this.pathFollowingCommand = new PathPlannerTrajectoryCommand(drive, path);
    this.pathMarkers = path.getMarkers();
    this.eventMap = eventMap;
  }

  /**
   * Create a FollowPathWithEvents command that will run a given path following command and run
   * commands associated with triggered event markers along the way.
   *
   * @param pathFollowingCommand The command that will run the path following. This acts like the
   *     deadline command in ParallelDeadlineGroup
   * @param pathMarkers The list of markers for the path that the path following command is
   *     following
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker. This SHOULD NOT contain any commands requiring the same subsystems as the path
   *     following command.
   */
  public FollowPathWithEvents(
      AutoCommand pathFollowingCommand,
      List<PathPlannerTrajectory.EventMarker> pathMarkers,
      Map<String, AutoCommand> eventMap) {
    this.pathFollowingCommand = pathFollowingCommand;
    this.pathMarkers = pathMarkers;
    this.eventMap = eventMap;
  }

  public void init() {
    isCompleted = false;

    currentCommands.clear();

    unpassedMarkers.clear();
    unpassedMarkers.addAll(pathMarkers);

    timer.reset();
    timer.start();

    pathFollowingCommand.init();
    currentCommands.put(pathFollowingCommand, true);
  }

  public void run() {
    for (Map.Entry<AutoCommand, Boolean> runningCommand : currentCommands.entrySet()) {
      if (!runningCommand.getValue()) {
        continue;
      }

      runningCommand.getKey().run();

      if (runningCommand.getKey().isCompleted()) {
        runningCommand.getKey().postComplete(false);
        runningCommand.setValue(false);
        if (runningCommand.getKey().equals(pathFollowingCommand)) {
          isCompleted = true;
        }
      }
    }

    double currentTime = timer.get();
    if (unpassedMarkers.size() > 0 && currentTime >= unpassedMarkers.get(0).timeSeconds) {
      PathPlannerTrajectory.EventMarker marker = unpassedMarkers.remove(0);

      for (String name : marker.names) {
        if (eventMap.containsKey(name)) {
          AutoCommand eventCommand = eventMap.get(name);

          for (Map.Entry<AutoCommand, Boolean> runningCommand : currentCommands.entrySet()) {
            if (!runningCommand.getValue()) {
              continue;
            }

            runningCommand.getKey().postComplete(true);
            runningCommand.setValue(false);
          }

          eventCommand.init();
          currentCommands.put(eventCommand, true);
        }
      }
    }
  }

  public void postComplete(boolean interrupted) {
    for (Map.Entry<AutoCommand, Boolean> runningCommand : currentCommands.entrySet()) {
      if (runningCommand.getValue()) {
        runningCommand.getKey().postComplete(true);
      }
    }
  }

  public boolean isCompleted() {
    return isCompleted;
  }
}

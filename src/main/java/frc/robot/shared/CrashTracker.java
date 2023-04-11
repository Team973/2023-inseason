package frc.robot.shared;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;
import lombok.experimental.Accessors;

/** Tracks start-up and caught crash events, logging them to a file which doesn't roll over */
@Accessors(prefix = "m_")
public class CrashTracker {
  @Getter private static boolean m_exceptionHappened = false;
  private static boolean m_matchDataFound = false;

  private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

  public static void logRobotInit() {
    logMarker("robot init");
  }

  public static void logTeleopInit() {
    logMarker("teleop init");
  }

  public static void logAutoInit() {
    logMarker("auto init");
  }

  public static void logDisabledInit() {
    logMarker("disabled init");
  }

  public static void logTestInit() {
    logMarker("test init");
  }

  public static void logThrowableCrash(Throwable e) {
    // TODO: emergency lights
    m_exceptionHappened = true;
    logMarker("Exception", e);
  }

  public static void logFMSData() {
    if (m_matchDataFound) {
      return;
    }

    var data =
        new FMSData(
            DriverStation.getEventName(),
            DriverStation.getMatchNumber(),
            DriverStation.getAlliance(),
            DriverStation.getMatchType(),
            DriverStation.getLocation(),
            DriverStation.getGameSpecificMessage());
    logMarker("FMS Data: " + data.toString());
    m_matchDataFound = true;
  }

  private static void logMarker(String mark) {
    logMarker(mark, null);
  }

  private static void logMarker(String mark, Throwable e) {
    if (!RobotBase.isSimulation()) {
      try (PrintWriter writer =
          new PrintWriter(new FileWriter("/home/lvuser/exception_log.txt", true))) {
        writer.print(RUN_INSTANCE_UUID.toString());
        writer.print(", ");
        writer.print(mark);
        writer.print(", ");
        writer.print(new Date().toString());

        if (e != null) {
          writer.print(", ");
          e.printStackTrace(writer);
        }
      } catch (IOException ie) {
        ie.printStackTrace();
      }
    } else {
      throw new RuntimeException(e);
    }
  }
}

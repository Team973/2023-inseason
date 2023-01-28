package frc.robot.greydash;

import static frc.robot.greydash.GreyDashConstants.*;

import java.util.HashSet;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;

/** The client for GreyDash. */
public final class GreyDashClient {

  private static final NetworkTableInstance ntCore = NetworkTableInstance.getDefault();
  private static final NetworkTable greyDashTable = ntCore.getTable(GREYDASH_TABLE);

  private static final NetworkTable autoTable = greyDashTable.getSubTable(AUTO_TABLE);
  private static final NetworkTable matchTable = greyDashTable.getSubTable(MATCH_TABLE);
  private static final NetworkTable devicesTable = greyDashTable.getSubTable(DEVICES_TABLE);
  private static final NetworkTable chartsTable = greyDashTable.getSubTable(CHARTS_TABLE);

  // Device Tables
  private static final NetworkTable gyroTable = devicesTable.getSubTable(GYRO_TABLE);

  // Auto Topics
  private static final StringArrayPublisher autoModes =
      autoTable.getStringArrayTopic(AUTO_MODES_TOPIC).publish();
  private static final StringSubscriber autoSelected =
      autoTable.getStringTopic(AUTO_SELECTED_TOPIC).subscribe("No Auto");

  // Match Topics
  private static final DoublePublisher matchTime =
      matchTable.getDoubleTopic(MATCH_TIME_TOPIC).publish();
  private static final StringPublisher matchMode =
      matchTable.getStringTopic(MATCH_MODE_TOPIC).publish();

  // Gyro Topics
  private static final DoublePublisher gyroAngle =
      gyroTable.getDoubleTopic(GYRO_ANGLE_TOPIC).publish();

  public static void setGyroAngle(double angle) {
    gyroAngle.set(angle);
  }

  // Charts publisher
  private static final StringArrayPublisher charts =
      chartsTable.getStringArrayTopic(CHARTS_TOPIC).publish();
  private static final HashSet<String> chartSet = new HashSet<>();

  /** Creates a new chart with the given name. */
  public static GreyDashChart createChart(final String name) {
    if (name == CHARTS_TOPIC) {
      throw new Error("Reserved chart name: " + CHARTS_TOPIC);
    }

    chartSet.add(name);
    charts.set(chartSet.toArray(new String[0]));

    return new GreyDashChart(chartsTable, name);
  }

  /**
   * Sets the available auto modes to appear in the dashboard's dropdown menu.
   *
   * @param modes The available auto modes.
   * @see #getAutoSelected()
   */
  public static void setAvailableAutoModes(final String... modes) {
    autoModes.set(modes);
  }

  /**
   * Gets the selected auto mode from the dashboard's dropdown menu.
   *
   * @return The selected auto mode.
   * @see #setAvailableAutoModes(String...)
   */
  public static String getAutoSelected() {
    return autoSelected.get();
  }

  /** Periodic update method. This should be called periodically to update the dashboard. */
  public static void update() {
    matchTime.set(DriverStation.getMatchTime());
    matchMode.set(getModeString());
  }
}

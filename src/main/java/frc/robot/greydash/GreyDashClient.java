package frc.robot.greydash;

import static frc.robot.greydash.GreyDashConstants.*;

import java.util.HashSet;

import frc.robot.AutoManager.AutoMode;
import frc.robot.shared.Constants.GamePiece;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;

/** The client for GreyDash. */
public final class GreyDashClient {

  private static final NetworkTableInstance m_ntCore = NetworkTableInstance.getDefault();
  private static final NetworkTable m_greyDashTable = m_ntCore.getTable(GREYDASH_TABLE);

  private static final NetworkTable m_autoTable = m_greyDashTable.getSubTable(AUTO_TABLE);
  private static final NetworkTable m_matchTable = m_greyDashTable.getSubTable(MATCH_TABLE);
  private static final NetworkTable m_devicesTable = m_greyDashTable.getSubTable(DEVICES_TABLE);
  private static final NetworkTable m_chartsTable = m_greyDashTable.getSubTable(CHARTS_TABLE);

  // Device Tables
  private static final NetworkTable m_gyroTable = m_devicesTable.getSubTable(GYRO_TABLE);

  // Auto Topics
  private static final StringArrayPublisher m_autoModes =
      m_autoTable.getStringArrayTopic(AUTO_MODES_TOPIC).publish();
  private static final StringSubscriber m_autoSelected =
      m_autoTable.getStringTopic(AUTO_SELECTED_TOPIC).subscribe(AutoMode.NoAuto.toString());

  private static final StringArrayPublisher m_gamePieces =
      m_autoTable.getStringArrayTopic(GAME_PIECES_TOPIC).publish();
  private static final StringSubscriber m_preloadSelected =
      m_autoTable.getStringTopic(GAME_PIECE_SELECTED_TOPIC).subscribe(GamePiece.None.toString());

  // Match Topics
  private static final DoublePublisher m_matchTime =
      m_matchTable.getDoubleTopic(MATCH_TIME_TOPIC).publish();
  private static final StringPublisher m_matchMode =
      m_matchTable.getStringTopic(MATCH_MODE_TOPIC).publish();

  // Gyro Topics
  private static final DoublePublisher m_gyroAngle =
      m_gyroTable.getDoubleTopic(GYRO_ANGLE_TOPIC).publish();

  public static void setGyroAngle(double angle) {
    m_gyroAngle.set(angle);
  }

  // Charts publisher
  private static final StringArrayPublisher m_charts =
      m_chartsTable.getStringArrayTopic(CHARTS_TOPIC).publish();
  private static final HashSet<String> m_chartSet = new HashSet<>();

  /** Creates a new chart with the given name. */
  public static GreyDashChart createChart(final String name) {
    if (name == CHARTS_TOPIC) {
      throw new Error("Reserved chart name: " + CHARTS_TOPIC);
    }

    m_chartSet.add(name);
    m_charts.set(m_chartSet.toArray(new String[0]));

    return new GreyDashChart(m_chartsTable, name);
  }

  /**
   * Sets the available auto modes to appear in the dashboard's dropdown menu.
   *
   * @param modes The available auto modes.
   * @see #getAutoSelected()
   */
  public static void setAvailableAutoModes(final String... modes) {
    m_autoModes.set(modes);
  }

  /**
   * Gets the selected auto mode from the dashboard's dropdown menu.
   *
   * @return The selected auto mode.
   * @see #setAvailableAutoModes(String...)
   */
  public static String getAutoSelected() {
    return m_autoSelected.get();
  }

  /**
   * Sets the available game piece preload options in the dashboard's dropdown menu.
   *
   * @param gamePieces The available game pieces.
   * @see #selectedGamePiece()
   */
  public static void availableGamePieces(final String... gamePieces) {
    m_gamePieces.set(gamePieces);
  }

  /**
   * Gets the selected preload from the dashboard's dropdown menu.
   *
   * @return The selected preload.
   * @see #availableGamePieces(String...)
   */
  public static String selectedGamePiece() {
    return m_preloadSelected.get();
  }

  /** Periodic update method. This should be called periodically to update the dashboard. */
  public static void update() {
    m_matchTime.set(DriverStation.getMatchTime());
    m_matchMode.set(getModeString());
  }
}

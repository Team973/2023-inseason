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
  private static final StringArrayPublisher m_availableAutoModes =
      m_autoTable.getStringArrayTopic(AVAILABLE_AUTO_MODES_TOPIC).publish();
  private static final StringSubscriber m_autoSelectedSubscriber =
      m_autoTable.getStringTopic(AUTO_SELECTED_TOPIC).subscribe(AutoMode.NoAuto.toString());
  private static final StringPublisher m_autoSelectedPublisher =
      m_autoTable.getStringTopic(AUTO_SELECTED_TOPIC).publish();

  private static final StringArrayPublisher m_availableGamePieces =
      m_autoTable.getStringArrayTopic(AVAILABLE_GAME_PIECES_TOPIC).publish();
  private static final StringSubscriber m_preloadSelectedSubscriber =
      m_autoTable.getStringTopic(GAME_PIECE_SELECTED_TOPIC).subscribe(GamePiece.None.toString());
  private static final StringPublisher m_preloadSelectedPublisher =
      m_autoTable.getStringTopic(GAME_PIECE_SELECTED_TOPIC).publish();

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
  public static void setAvailableAutoModes(final AutoMode... modes) {
    String[] modeStrings = new String[modes.length];
    for (int i = 0; i < modes.length; i++) {
      modeStrings[i] = modes[i].toString();
    }
    m_availableAutoModes.set(modeStrings);
  }

  /**
   * Sets the selected auto to GreyDash from the Robot. Used to initialize the default dropdown
   * state.
   *
   * @param auto The default auto mode.
   * @see #getAutoSelected()
   */
  public static void setSelectedAuto(final AutoMode auto) {
    m_autoSelectedPublisher.set(auto.toString());
  }

  /**
   * Gets the selected auto mode from the dashboard's dropdown menu.
   *
   * @return The selected auto mode.
   * @see #setAvailableAutoModes(String...)
   */
  public static AutoMode getAutoSelected() {
    return AutoMode.valueOf(m_autoSelectedSubscriber.get());
  }

  /**
   * Sets the available game piece preload options in the dashboard's dropdown menu.
   *
   * @param gamePieces The available game pieces.
   * @see #getSelectedGamePiece()
   */
  public static void setAvailableGamePieces(final GamePiece... gamePieces) {
    String[] gamePieceStrings = new String[gamePieces.length];
    for (int i = 0; i < gamePieces.length; i++) {
      gamePieceStrings[i] = gamePieces[i].toString();
    }
    m_availableGamePieces.set(gamePieceStrings);
  }

  /**
   * Gets the selected preload from the dashboard's dropdown menu.
   *
   * @return The selected preload.
   * @see #setAvailableGamePieces(String...)
   */
  public static GamePiece getSelectedGamePiece() {
    return GamePiece.valueOf(m_preloadSelectedSubscriber.get());
  }

  /**
   * Sets the selected preload to GreyDash from the Robot. Used to initialize the default dropdown.
   *
   * @param gamePiece The default preload.
   */
  public static void setSelectedGamePiece(final GamePiece gamePiece) {
    m_preloadSelectedPublisher.set(gamePiece.toString());
  }

  /** Periodic update method. This should be called periodically to update the dashboard. */
  public static void update() {
    m_matchTime.set(DriverStation.getMatchTime());
    m_matchMode.set(getModeString());
  }
}

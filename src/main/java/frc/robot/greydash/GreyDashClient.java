package frc.robot.greydash;

import static frc.robot.greydash.GreyDashConstants.*;

import java.util.Arrays;
import java.util.HashSet;

import frc.robot.AutoManager.AutoMode;
import frc.robot.AutoManager.AutoSide;
import frc.robot.subsystems.Superstructure.GamePiece;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArraySubscriber;
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
  private static final StringSubscriber m_gamePieceSelectedSubscriber =
      m_autoTable.getStringTopic(GAME_PIECE_SELECTED_TOPIC).subscribe(GamePiece.None.toString());
  private static final StringPublisher m_gamePieceSelectedPublisher =
      m_autoTable.getStringTopic(GAME_PIECE_SELECTED_TOPIC).publish();

  private static final StringArrayPublisher m_availableAutoSides =
      m_autoTable.getStringArrayTopic(AVAILABLE_AUTO_SIDES_TOPIC).publish();
  private static final StringSubscriber m_autoSideSelectedSubscriber =
      m_autoTable.getStringTopic(AUTO_SIDE_SELECTED_TOPIC).subscribe(AutoSide.Left.toString());
  private static final StringPublisher m_autoSideSelectedPublisher =
      m_autoTable.getStringTopic(AUTO_SIDE_SELECTED_TOPIC).publish();

  private static final StringArraySubscriber m_stagingSelectionSubscriber =
      m_autoTable.getStringArrayTopic(STAGING_SELECTION_TOPIC).subscribe(new String[0]);
  private static final StringArrayPublisher m_stagingSelectionPublisher =
      m_autoTable.getStringArrayTopic(STAGING_SELECTION_TOPIC).publish();

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
    m_availableAutoModes.set(Arrays.stream(modes).map(Enum::toString).toArray(String[]::new));
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
    m_availableGamePieces.set(Arrays.stream(gamePieces).map(Enum::toString).toArray(String[]::new));
  }

  /**
   * Gets the selected preload from the dashboard's dropdown menu.
   *
   * @return The selected preload.
   * @see #setAvailableGamePieces(String...)
   */
  public static GamePiece getSelectedGamePiece() {
    return GamePiece.valueOf(m_gamePieceSelectedSubscriber.get());
  }

  /**
   * Sets the selected preload to GreyDash from the Robot. Used to initialize the default dropdown.
   *
   * @param gamePiece The default preload.
   */
  public static void setSelectedGamePiece(final GamePiece gamePiece) {
    m_gamePieceSelectedPublisher.set(gamePiece.toString());
  }

  /**
   * Sets the available auto side options in the dashboard's dropdown menu.
   *
   * @param autoSides The available auto sides.
   * @see #getSelectedAutoSide()
   */
  public static void setAvailableAutoSides(final AutoSide... autoSides) {
    m_availableAutoSides.set(
        Arrays.stream(autoSides).map((side) -> side.toString()).toArray(String[]::new));
  }

  /**
   * Gets the selected auto side from the dashboard's dropdown menu.
   *
   * @return The selected auto side.
   * @see #setAvailableAutoSides(String...)
   */
  public static AutoSide getSelectedAutoSide() {
    return AutoSide.valueOf(m_autoSideSelectedSubscriber.get());
  }

  /**
   * Sets the selected auto side to GreyDash from the Robot. Used to initialize the default
   * dropdown.
   *
   * @param autoSide The default auto side.
   */
  public static void setSelectedAutoSide(final AutoSide autoSide) {
    m_autoSideSelectedPublisher.set(autoSide.toString());
  }

  /**
   * Set the selected staging GamePieces to GreyDash from the Robot. Used to initialize the default
   * dropdown.
   */
  public static void setSelectedStagingGamePieces(final GamePiece... gamePieces) {
    if (gamePieces.length != 4) {
      throw new IllegalArgumentException("Length of staging GamePieces must be equal to 4.");
    }
    m_stagingSelectionPublisher.set(
        Arrays.stream(gamePieces).map((piece) -> piece.toString()).toArray(String[]::new));
  }

  /**
   * Gets the selected staging GamePieces from the dashboard's dropdown menu.
   *
   * @return The selected staging GamePieces.
   */
  public static GamePiece[] getSelectedStagingGamePieces() {
    return Arrays.stream(m_stagingSelectionSubscriber.get())
        .map((piece) -> GamePiece.valueOf(piece))
        .toArray(GamePiece[]::new);
  }

  /** Periodic update method. This should be called periodically to update the dashboard. */
  public static void update() {
    m_matchTime.set(DriverStation.getMatchTime());
    m_matchMode.set(getModeString());
  }
}

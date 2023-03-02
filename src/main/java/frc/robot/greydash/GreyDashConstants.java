package frc.robot.greydash;

import edu.wpi.first.wpilibj.DriverStation;

public final class GreyDashConstants {
  // Basic tables
  public static final String GREYDASH_TABLE = "GreyDash";
  public static final String AUTO_TABLE = "auto";
  public static final String MATCH_TABLE = "match";
  public static final String DEVICES_TABLE = "devices";
  public static final String CHARTS_TABLE = "charts";

  // Device Tables
  public static final String GYRO_TABLE = "gyro";

  ////////////
  // Topics //
  ////////////

  // Auto
  public static final String AVAILABLE_AUTO_MODES_TOPIC = "availableModes";
  public static final String AUTO_SELECTED_TOPIC = "selectedMode";
  public static final String AVAILABLE_GAME_PIECES_TOPIC = "availableGamePieces";
  public static final String GAME_PIECE_SELECTED_TOPIC = "selectedPreload";
  public static final String AVAILABLE_AUTO_SIDES_TOPIC = "availableAutoSides";
  public static final String AUTO_SIDE_SELECTED_TOPIC = "selectedAutoSide";

  // Match
  public static final String MATCH_TIME_TOPIC = "time";
  public static final String MATCH_MODE_TOPIC = "mode";

  // Gyro
  public static final String GYRO_ANGLE_TOPIC = "angle";

  // Charts
  public static final String CHARTS_TOPIC = "list";

  public static String getModeString() {
    if (DriverStation.isDisabled()) {
      return "Disabled";
    } else if (DriverStation.isAutonomous()) {
      return "Auto";
    } else if (DriverStation.isTeleop()) {
      return "Teleop";
    } else if (DriverStation.isTest()) {
      return "Test";
    } else {
      return "unknown";
    }
  }
}

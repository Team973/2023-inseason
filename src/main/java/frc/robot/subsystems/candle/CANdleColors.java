package frc.robot.subsystems.candle;

import frc.robot.subsystems.Superstructure.GamePiece;

public final class CANdleColors {
  public static final RGBColor cube = new RGBColor(170, 0, 255);
  public static final RGBColor cone = new RGBColor(255, 150, 0);
  public static final RGBColor gotIt = new RGBColor(0, 255, 0);
  public static final RGBColor emergency = new RGBColor(255, 0, 0);
  public static final RGBColor autoWaiting = new RGBColor(0, 0, 255);
  public static final RGBColor balance = new RGBColor(244, 140, 0);
  public static final RGBColor off = new RGBColor(0, 0, 0);

  public static RGBColor getColorFromGamePiece(GamePiece gamePiece) {
    switch (gamePiece) {
      case Cube:
        return cube;
      case Cone:
        return cone;
      default:
        return off;
    }
  }
}

package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Wrist.WristPreset;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Superstructure implements Subsystem {
  /** Game Piece options. */
  public enum GamePiece {
    Cube,
    Cone,
    None
  }

  @Setter @Getter private static GamePiece m_currentGamePiece = GamePiece.None;

  private final Wrist m_wrist = new Wrist();
  private final Elevator m_elevator = new Elevator();

  private boolean m_isStowed = true;

  public void wristStow() {
    if (!m_isStowed) {
      m_wrist.setPreset(WristPreset.PreStow);
      if (m_elevator.getBottomHall()) {
        m_wrist.setPreset(WristPreset.Stow);
        m_isStowed = true;
      }
    }
  }

  public void dashboardUpdate() {}

  public void update() {
    if (!m_elevator.getBottomHall()) {
      m_isStowed = false;
    }
  }

  public void reset() {}
}

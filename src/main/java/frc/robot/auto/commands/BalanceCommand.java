package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import lombok.NonNull;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class BalanceCommand extends AutoCommand {
  private static final double ANGLE_TOLERANCE_DEG = 2.0;

  private static enum Direction {
    Roll,
    Pitch
  }

  private static Direction m_direction;

  @NonNull private final Drive m_drive;
  private final double m_targetMsec;

  public void init() {
    setTargetMsec(m_targetMsec);
  }

  public void run() {
    double yaw = m_drive.getPigeon().getNormalizedYaw().getDegrees();
    if (Math.abs(yaw - 90.0) < 30.0 || Math.abs(yaw - 270.0) < 30.0) {
      m_drive.balanceDriveRoll();
      m_direction = Direction.Roll;
    } else {
      m_drive.balanceDrivePitch();
      m_direction = Direction.Pitch;
    }
  }

  public boolean isCompleted() {
    boolean completed = false;
    switch (m_direction) {
      case Roll:
        completed = Math.abs(m_drive.getPigeon().getRoll().getDegrees()) < ANGLE_TOLERANCE_DEG;
        break;
      case Pitch:
        completed = Math.abs(m_drive.getPigeon().getPitch().getDegrees()) < ANGLE_TOLERANCE_DEG;
        break;
      default:
        break;
    }
    return completed;
  }

  public void postComplete(boolean interrupted) {}
}

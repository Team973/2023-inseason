package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RotationControl;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class SpinInPlaceCommand extends AutoCommand {

  private final Drive m_drive;
  private final Rotation2d m_angle;

  @Override
  public void init() {
    m_drive.setRotationControl(RotationControl.ClosedLoop);
    m_drive.setTargetRobotAngle(m_angle);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return Math.abs(m_drive.getPigeon().getNormalizedYaw().minus(m_angle).getDegrees()) <= 2.0;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}

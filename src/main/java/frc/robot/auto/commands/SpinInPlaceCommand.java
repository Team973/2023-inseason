package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RotationControl;

import edu.wpi.first.math.geometry.Rotation2d;

public class SpinInPlaceCommand extends AutoCommand {

  private final Drive m_drive;
  private final double m_angleDegrees;

  public SpinInPlaceCommand(Drive drive, double angleDegrees) {
    m_drive = drive;
    m_angleDegrees = angleDegrees;
  }

  @Override
  public void init() {
    m_drive.setRotationControl(RotationControl.ClosedLoop);
    m_drive.setTargetRobotAngle(Rotation2d.fromDegrees(m_angleDegrees));
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return Math.abs(m_drive.getPigeon().getNormalizedYaw().getDegrees() - m_angleDegrees) <= 2.0;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}

package frc.robot.shared;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class SwerveModuleConfig {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final double angleOffset;
}

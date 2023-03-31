package frc.robot.shared.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;

public class GearedMechanism {
  @Setter @Getter private double m_gearRatio;

  /**
   * Create a GearedMechanism with a gear ratio.
   *
   * <p>For example, if the pinion is 10 teeth and the gear is 50 teeth, the gear ratio is 1:5 and
   * therefore the value is 0.2.
   *
   * @param gearRatio Gear ratio (input / output, driving:driven)
   */
  public GearedMechanism(double gearRatio) {
    m_gearRatio = gearRatio;
  }

  /**
   * Convert through the gear ratio to get the output's respective Rotation2d.
   *
   * @param rotorRotation The Rotation2d on the output to convert.
   * @return The current output Rotation2d of the TalonFX through the gear ratio.
   */
  public Rotation2d getOutputRotationFromRotorRotation(Rotation2d rotorRotation) {
    return rotorRotation.times(m_gearRatio);
  }

  /**
   * Convert through the gear ratio to get the rotor's respective Rotation2d.
   *
   * @param outputRotation The Rotation2d on the output to convert.
   * @return The Rotation2d on the rotor.
   */
  public Rotation2d getRotorRotationFromOutputRotation(Rotation2d outputRotation) {
    return outputRotation.div(m_gearRatio);
  }
}

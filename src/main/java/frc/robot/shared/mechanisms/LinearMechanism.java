package frc.robot.shared.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

public class LinearMechanism extends GearedMechanism {
  private final double m_diameter;

  /**
   * Create a LinearMechanism with a gear ratio and wheel/pitch diameter.
   *
   * <p>For example, if the pinion is 10 teeth and the gear is 50 teeth, the gear ratio is 1:5 and
   * therefore the value is 0.2.
   *
   * @param gearRatio Gear ratio (input / output, driving:driven)
   * @param diameter Wheel/pitch diameter. Units do not matter as long as they are consistent.
   */
  public LinearMechanism(double gearRatio, double diameter) {
    super(gearRatio);
    m_diameter = diameter;
  }

  /**
   * Get the circumference of the wheel/sprocket.
   *
   * @return The circumference of the wheel/sprocket. Units are the same as the wheel/pitch
   *     diameter.
   */
  public double getCircumference() {
    return m_diameter * Math.PI;
  }

  /**
   * Get the output distance from the rotor rotation.
   *
   * @param outputRotation The Rotation2d on the output to convert.
   * @return The current output distance through the gear ratio.
   */
  public double getOutputDistanceFromOutputRotation(Rotation2d outputRotation) {
    return outputRotation.getRotations() * getCircumference();
  }

  /**
   * Get the output distance from the rotor rotation.
   *
   * @param rotorRotation The Rotation2d on the rotor to convert.
   * @return The current output distance through the gear ratio.
   */
  public double getOutputDistanceFromRotorRotation(Rotation2d rotorRotation) {
    return getOutputDistanceFromOutputRotation(getOutputRotationFromRotorRotation(rotorRotation));
  }

  /**
   * Get the output Rotation2d from the output distance.
   *
   * @param outputDistance The distance on the output to convert.
   * @return The current output Rotation2d through the gear ratio.
   */
  public Rotation2d getOutputRotationFromOutputDistance(double outputDistance) {
    return Rotation2d.fromRotations(outputDistance / getCircumference());
  }

  /**
   * Get the rotor rotation from the output distance.
   *
   * @param outputDistance The distance on the output to convert.
   * @return The current rotor rotation through the gear ratio.
   */
  public Rotation2d getRotorRotationFromOutputDistance(double outputDistance) {
    return getRotorRotationFromOutputRotation(getOutputRotationFromOutputDistance(outputDistance));
  }
}

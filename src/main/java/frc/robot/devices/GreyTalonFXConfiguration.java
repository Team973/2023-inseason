package frc.robot.devices;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import lombok.EqualsAndHashCode;

/**
 * A configuration for a GreyTalonFX. Extends the TalonFXConfiguration and provides equals,
 * hashCode, and a copy constructor.
 */
@EqualsAndHashCode(callSuper = true)
public class GreyTalonFXConfiguration extends TalonFXConfiguration {
  /** Creates a new GreyTalonFXConfiguration. Do you want to use m_motor.getConfig() instead? */
  public GreyTalonFXConfiguration() {
    super();
  }

  /**
   * Creates a new GreyTalonFXConfiguration by copying another.
   *
   * @param other The other GreyTalonFXConfiguration to copy.
   */
  public GreyTalonFXConfiguration(GreyTalonFXConfiguration other) {
    super();
    this.deserialize(other.serialize());
  }
}

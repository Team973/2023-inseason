package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorPresetCommand extends AutoCommand {
  private final Elevator m_elevator;

  private final Elevator.Preset m_preset;
  private final double m_timeout;

  public ElevatorPresetCommand(Elevator elevator, Elevator.Preset preset, double timeout) {
    m_elevator = elevator;
    m_preset = preset;
    m_timeout = timeout;
  }

  @Override
  public void init() {
    setTargetMsec(m_timeout);
    m_elevator.setElevatorState(ElevatorState.ClosedLoop);
    m_elevator.setPreset(m_preset);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return m_elevator.isAtTargetHeight();
  }

  public void postComplete(boolean interrupted) {}
}

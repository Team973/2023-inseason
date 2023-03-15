// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.WristPreset;
import frc.robot.subsystems.Claw.WristState;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class WristPresetCommand extends AutoCommand {
  private final Claw m_claw;
  private final WristPreset m_preset;
  private final double m_minAngleVelocity;
  private final double m_targetMsec;

  // Called when the command is initially scheduled.
  public void init() {
    m_claw.setWristState(WristState.ClosedLoop);
    setTargetMsec(m_targetMsec);
    m_claw.setWristPreset(m_preset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void run() {}

  public boolean isCompleted() {
    return m_claw.isAtAngle() && Math.abs(m_claw.getWristVelocity()) < m_minAngleVelocity;
  }

  public void postComplete(boolean interrupted) {}
}

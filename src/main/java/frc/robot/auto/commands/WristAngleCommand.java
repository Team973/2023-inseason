// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.WristState;

public class WristAngleCommand extends AutoCommand {

  private final double m_targetMsec;

  private final double m_angle;
  private final Claw m_claw;
  /** Creates a new WristCommand. */
  public WristAngleCommand(Claw claw, double angle, double targetMsec) {
    this.m_targetMsec = targetMsec;
    this.m_angle = angle;
    this.m_claw = claw;
  }

  // Called when the command is initially scheduled.
  public void init() {
    m_claw.setWristState(WristState.ClosedLoop);
    setTargetMsec(m_targetMsec);
    m_claw.setWristTargetAngle(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void run() {}

  public boolean isCompleted() {
    // TODO Auto-generated method stub
    return hasElapsed() || m_claw.isAtAngle();
  }

  public void postComplete() {}
}

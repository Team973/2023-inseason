package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Claw.IntakeState;

public class PickupGamePiece extends AutoCommand {

  private final Claw m_claw;
  private final GamePiece m_gamePiece;

  private final double m_timeout;

  public PickupGamePiece(Claw claw, GamePiece gamePiece, double TimeOut) {
    m_claw = claw;
    m_gamePiece = gamePiece;
    m_timeout = TimeOut;
  }

  public void init() {
    // TODO Auto-generated method stub
    setTargetMsec(m_timeout);
  }

  public void run() {
    // TODO Auto-generated method stub
    m_claw.setCurrentGamePiece(m_gamePiece);
    m_claw.setIntakeState(IntakeState.In);
  }

  public boolean isCompleted() {
    // TODO Auto-generated method stub
    return hasElapsed();
  }

  public void postComplete() {
    m_claw.setIntakeState(IntakeState.Hold);
  }
}

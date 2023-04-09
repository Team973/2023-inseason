package frc.robot.shared;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import lombok.Data;

@Data
public class FMSData {
  private final String m_eventName;
  private final int m_matchNumber;
  private final Alliance m_alliance;
  private final MatchType m_matchType;
  private final int m_location;
  private final String m_gameSpecificMessage;
}

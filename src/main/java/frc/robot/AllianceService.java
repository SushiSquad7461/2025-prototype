package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceService {
    private static Alliance alliance;
    public static Alliance getAlliance() {
        if(alliance != null) {
            alliance = DriverStation.getAlliance().get();
        }
        return alliance;
    }
}

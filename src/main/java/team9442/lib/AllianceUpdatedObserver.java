package team9442.lib;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface AllianceUpdatedObserver {

    public void onAllianceFound(Alliance alliance);
}

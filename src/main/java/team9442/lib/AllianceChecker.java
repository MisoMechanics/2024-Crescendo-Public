package team9442.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AllianceChecker {
    private final List<AllianceUpdatedObserver> observers = new ArrayList<>();
    private Optional<Alliance> alliance = DriverStation.getAlliance();

    public void registerObserver(AllianceUpdatedObserver observer) {
        observers.add(observer);
    }

    public void registerObservers(AllianceUpdatedObserver... addObservers) {
        for (final AllianceUpdatedObserver o : addObservers) {
            observers.add(o);
        }
    }

    public void periodic() {
        alliance = DriverStation.getAlliance();

        alliance.ifPresent(color -> observers.forEach(observer -> observer.onAllianceFound(color)));
    }
}

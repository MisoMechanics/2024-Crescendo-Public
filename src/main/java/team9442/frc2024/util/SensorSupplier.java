package team9442.frc2024.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team9442.lib.VirtualSubsystem;

public class SensorSupplier extends VirtualSubsystem implements BooleanSupplier {

    private final Trigger feederSensor;
    private final DoubleSupplier feederVelocity;
    private final DoubleSupplier feederCurrent;

    private boolean velocitySpikeDown = false;
    private boolean currentSpikeUp = true;

    LinearFilter velocityFilter = LinearFilter.highPass(0.1, 0.02);
    LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);

    private boolean sensorBroken = false;

    public SensorSupplier(
            Trigger feederSensor, DoubleSupplier feederVelocity, DoubleSupplier feederCurrent) {
        this.feederSensor = feederSensor;
        this.feederVelocity = feederVelocity;
        this.feederCurrent = feederCurrent;
    }

    public void setBroken() {
        this.sensorBroken = true;
    }

    public void setGood() {
        this.sensorBroken = false;
    }

    public void periodic() {
        velocitySpikeDown = velocityFilter.calculate(feederVelocity.getAsDouble()) < 5;
        currentSpikeUp = currentFilter.calculate(feederCurrent.getAsDouble()) > 15;
    }

    @Override
    public boolean getAsBoolean() {
        if (!sensorBroken) {
            return feederSensor.getAsBoolean();
        }

        return velocitySpikeDown && currentSpikeUp;
    }
}

package team9442.frc2024.constants;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import team9442.lib.ServoConstants;
import team9442.lib.SparkmaxUtil;
import team9442.lib.SparkmaxUtil.Data;
import team9442.lib.SparkmaxUtil.Sensor;

public final class IntakeConstants {

    public static final CANSparkMax kIntake =
            new CANSparkMax(GlobalConstants.IntakeIds.kIntakeId, MotorType.kBrushless);
    public static final CANSparkMax kIndexer =
            new CANSparkMax(GlobalConstants.IntakeIds.kIndexerId, MotorType.kBrushless);

    public static final LaserCan kIntakeSensor = new LaserCan(20);
    public static final Trigger kIntakeSensorTrigger =
            new Trigger(
                    () -> {
                        var measurement = kIntakeSensor.getMeasurement();
                        return (measurement == null ? 0 : measurement.distance_mm) < 10;
                    });
    public static final boolean kIntakeInverted = false;
    public static final boolean kIndexerInverted = false;

    public static final double kOutputRotationsPerMotorRotation = 1.0;
    public static final double kOutputMetersPerMotorRotation =
            Math.PI * Units.inchesToMeters(2) * kOutputRotationsPerMotorRotation;

    public static final double kOutputMetersPerSecondPerMotorRPM =
            kOutputMetersPerMotorRotation / 60.0;

    public static final double kIntakeVelocity = 0;

    public static final ServoConstants kConstants =
            new ServoConstants.Builder()
                    .withVelocityConversion(kOutputMetersPerSecondPerMotorRPM)
                    .withVelocityTolerance(0.5)
                    .build();

    static {
        SparkmaxUtil.prepareForConfig(kIntake, "intake horizontal rollers");
        SparkmaxUtil.prepareForConfig(kIndexer, "intake vertical rollers");

        kIntake.setInverted(kIntakeInverted);
        kIndexer.setInverted(kIndexerInverted);

        kIntake.setSmartCurrentLimit(60);
        kIndexer.setSmartCurrentLimit(40);

        try {
            kIntakeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            kIntakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(6, 6, 6, 6));
            kIntakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            //            throw new RuntimeException(e);
        }

        SparkmaxUtil.configPID(kIntake, 0, 0.1, 0, "horizontal rollers");
        SparkmaxUtil.configPID(kIndexer, 0, 0.1, 0, "vertical rollers");

        SparkmaxUtil.configureFrameStrategy(
                kIntake,
                Set.of(Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false);

        SparkmaxUtil.configureFrameStrategy(
                kIndexer,
                Set.of(Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false);
    }

    private IntakeConstants() {}
}

package team9442.frc2024.constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import java.util.Set;
import team9442.lib.ServoConstants;
import team9442.lib.SparkmaxUtil;
import team9442.lib.SparkmaxUtil.Data;
import team9442.lib.SparkmaxUtil.Sensor;

public final class AmpPivotConstants {
    public static final CANSparkMax kMaster =
            new CANSparkMax(GlobalConstants.AmpPivotIds.kMasterId, MotorType.kBrushless);

    public static final double kOutputRotationsPerMotorRotation =
            12.0 / 48.0 * 18.0 / 32.0 * 16.0 / 48.0;
    public static final double kOutputDegreesPerMotorRotation =
            360 * kOutputRotationsPerMotorRotation;

    public static final ServoConstants kConstants =
            new ServoConstants.Builder()
                    .withPositionConversion(kOutputDegreesPerMotorRotation)
                    .withResetPosition(250.0)
                    .withMinPosition(20)
                    .withMaxPosition(250.0)
                    .withPositionTolerance(8)
                    .withSimpleMotorFeedforward((position, velocity) -> 0.0)
                    .build();

    public static final double kHandoffAngle = 20;
    public static final double kAmpScoreAngle = 150;
    public static final double kAmpIntakeAngle = 155;
    public static final double kAmpLoadingStationAngle = 117;

    static {
        SparkmaxUtil.prepareForConfig(kMaster, "amp pivot");
        kMaster.setIdleMode(IdleMode.kBrake);
        SparkmaxUtil.configPID(kMaster, 0, 0.1, 0, "amp pivot");

        kMaster.setSmartCurrentLimit(60);

        SparkmaxUtil.configureFrameStrategy(
                kMaster,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT, Data.CURRENT),
                Set.of(Sensor.INTEGRATED),
                false);
    }

    private AmpPivotConstants() {}
}

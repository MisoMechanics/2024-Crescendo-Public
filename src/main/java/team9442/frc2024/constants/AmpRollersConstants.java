package team9442.frc2024.constants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import java.util.Set;
import team9442.lib.ServoConstants;
import team9442.lib.SparkmaxUtil;
import team9442.lib.SparkmaxUtil.Data;
import team9442.lib.SparkmaxUtil.Sensor;

public class AmpRollersConstants {

    public static final CANSparkMax kMaster =
            new CANSparkMax(GlobalConstants.AmpRollersIds.kMasterId, MotorType.kBrushless);

    public static final double kOutputRotationsPerMotorRotation = 18.0 / 48.0;
    public static final double kOutputMetersPerMotorRotation =
            Math.PI * Units.inchesToMeters(2) * kOutputRotationsPerMotorRotation;

    public static final double kOutputMetersPerSecondPerMotorRPM =
            kOutputMetersPerMotorRotation / 60.0;

    public static final ServoConstants kConstants =
            new ServoConstants.Builder()
                    .withVelocityConversion(kOutputMetersPerSecondPerMotorRPM)
                    .build();

    public static final double kCurrentHold = 0;
    public static final double kScoreVelocity = 0;

    static {
        SparkmaxUtil.prepareForConfig(kMaster, "amp rollers");

        SparkmaxUtil.configPID(kMaster, 0, 0.1, 0, "amp rollers");

        SparkmaxUtil.configureFrameStrategy(
                kMaster,
                Set.of(Data.VELOCITY, Data.APPLIED_OUTPUT, Data.CURRENT),
                Set.of(Sensor.INTEGRATED),
                false);
    }

    private AmpRollersConstants() {}
}

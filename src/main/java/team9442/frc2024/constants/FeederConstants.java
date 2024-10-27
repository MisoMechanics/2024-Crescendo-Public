package team9442.frc2024.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team9442.lib.ServoConstants;

public final class FeederConstants {

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.FeederIds.kMasterId, GlobalConstants.kCANbusName);
    public static final DigitalInput kFeederSensor =
            new DigitalInput(GlobalConstants.FeederIds.kFeederSensorPort);
    public static final DigitalInput kStagingSensor =
            new DigitalInput(GlobalConstants.FeederIds.kStagingSensorPort);

    public static final Trigger kFeederSensorTrigger =
            new Trigger(() -> !FeederConstants.kFeederSensor.get());
    public static final Trigger kStagingSensorTrigger =
            new Trigger(() -> !FeederConstants.kStagingSensor.get());

    public static final double kOutputRotationsPerMotorRotation = 18.0 / 36.0;
    public static final double kOutputMetersPerMotorRotation =
            Math.PI * Units.inchesToMeters(2) * kOutputRotationsPerMotorRotation;

    public static final double kOutputMetersPerSecondPerMotorRPM =
            kOutputMetersPerMotorRotation / 60.0;

    public static final double kFeedVelocity = 0;

    public static final ServoConstants kConstants =
            new ServoConstants.Builder()
                    .withVelocityConversion(kOutputMetersPerSecondPerMotorRPM)
                    .withVelocityTolerance(0.2)
                    .build();

    static {
        TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
        kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMaster.getConfigurator().apply(kMasterConfig);
    }

    private FeederConstants() {}
}

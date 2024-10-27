package team9442.frc2024.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import team9442.lib.ServoConstants;

public final class ShooterPivotConstants {

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.ShooterPivotIds.kMasterId, GlobalConstants.kCANbusName);

    public static final double kOutputRotationsPerMotorRotation =
            12.0 / 48.0 * 18.0 / 54.0 * 18.0 / 48.0 * 14.0 / 48.0;
    public static final double kOutputDegreesPerMotorRotation = 360;

    public static final double kOutputRotationsPerSecondPerMotorVelocity =
            kOutputRotationsPerMotorRotation;

    public static final double kHandoffAngle = 67;
    public static final double kSpeakerFarTrussScoreAngle = 15;
    public static final double kSpeakerPodiumScoreAngle = 23;
    public static final double kSpeakerFenderScoreAngle = 58;
    public static final double kGroundIntakeAngle = 45;
    public static final double kLoadingIntakeAngle = 117;

    public static final boolean kInverted = true;

    public static final ServoConstants kConstants =
            new ServoConstants.Builder()
                    .withPositionConversion(360)
                    .withVelocityConversion(360)
                    .withMinPosition(16.45)
                    .withResetPosition(16.45)
                    .withMaxPosition(64.75)
                    .withPositionTolerance(1)
                    .withSimpleMotorFeedforward((position, velocity) -> 0.0)
                    .withHomingCurrentAmps(15)
                    .withHomingSpeed(-0.2)
                    .build();

    static {
        final TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // 0 -> 0.2
        configuration.Slot0.kA = 0.1;
        configuration.Slot0.kD = 0.1;
        configuration.Slot0.kG = 0.37;
        configuration.Slot0.kP = 160;
        configuration.Slot0.kS = 0.6;
        configuration.Slot0.kV = 7;

        configuration.MotionMagic.MotionMagicCruiseVelocity = 3;
        configuration.MotionMagic.MotionMagicAcceleration = 4;

        configuration.CurrentLimits.SupplyCurrentLimit = 30;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        configuration.CurrentLimits.SupplyTimeThreshold = 1.5;

        configuration.Feedback.SensorToMechanismRatio = 1 / kOutputRotationsPerMotorRotation;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kConstants.maxPosition / 360;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kConstants.minPosition / 360;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kMaster.getConfigurator().apply(configuration, 1);
    }

    private ShooterPivotConstants() {}
}

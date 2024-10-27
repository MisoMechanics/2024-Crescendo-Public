package team9442.frc2024.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import team9442.lib.ServoConstants;

public final class ShooterRollersConstants {

    public static final TalonFX kPortSide =
            new TalonFX(GlobalConstants.ShooterRollersIds.kPortId, GlobalConstants.kCANbusName);
    public static final TalonFX kStarboardSide =
            new TalonFX(
                    GlobalConstants.ShooterRollersIds.kStarboardId, GlobalConstants.kCANbusName);

    public static final double kOutputRotationsPerMotorRotation = 36.0 / 24.0;
    public static final double kOutputMetersPerMotorRotation =
            Math.PI * Units.inchesToMeters(3) * kOutputRotationsPerMotorRotation;

    public static final ServoConstants kConstants =
            new ServoConstants.Builder()
                    .withVelocityConversion(kOutputMetersPerMotorRotation)
                    .withVelocityTolerance(0.6)
                    .build();

    /** m/s */
    public static final double kSpeakerScoreOpenLoopProtected = 1;

    public static final double kSpeakerScoreOpenLoopFender = 0.6;
    public static final double kSpeakerScoreClosedLoopFender = 14.0;
    public static final double kSpeakerScoreClosedLoopProtected = 20.0;
    public static final double kSpeakerScoreClosedLoopFarTruss = 24.0;

    static {
        TalonFXConfiguration portSideConfig = new TalonFXConfiguration();
        TalonFXConfiguration starboardConfig = new TalonFXConfiguration();

        portSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        portSideConfig.Slot0.kS = 0.019;
        portSideConfig.Slot0.kV = 0.0121276;
        portSideConfig.Slot0.kP = 0.06;
        portSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        portSideConfig.CurrentLimits.StatorCurrentLimit = 150;
        portSideConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kPortSide.getConfigurator().apply(portSideConfig, 1);

        starboardConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        starboardConfig.Slot0.kS = 0.021;
        starboardConfig.Slot0.kV = 0.0121276;
        starboardConfig.Slot0.kP = 0.06;
        starboardConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        starboardConfig.TorqueCurrent.PeakForwardTorqueCurrent = 150;
        starboardConfig.CurrentLimits.StatorCurrentLimit = 150;
        starboardConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kStarboardSide.getConfigurator().apply(starboardConfig, 1);
    }

    private ShooterRollersConstants() {}
}

package team9442.frc2024.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import monologue.Logged;
import team9442.lib.PoseUtil;
import team9442.lib.VirtualSubsystem;

public class LookaheadCalculator extends VirtualSubsystem implements Logged {

    public static record RawAimingParameters(double yawToTarget, double rangeToTargetMeters) {}

    public static record LookaheadAimingParameters(
            double effectiveYawSetpoint,
            double effectiveRangeToTarget,
            double effectiveYawFeedforwardVelocityDegPS,
            double effectiveRangeFeedforwardMpS,
            double rawYawSetpoint,
            double rawRangeToTarget) {}

    public static Rotation2d kOneEighty = Rotation2d.fromDegrees(180);

    private final Supplier<ChassisSpeeds> getFieldRelativeSpeeds;
    private final Supplier<Rotation2d> getRobotYaw;
    private final Supplier<RawAimingParameters> getLookaheadInput;
    private final BooleanSupplier allowShootOnTheMove;
    private final double shotTime;

    public static final LookaheadAimingParameters kEmptyOutput =
            new LookaheadAimingParameters(0, 0, 0, 0, 0, 0);
    private LookaheadAimingParameters latestOutput = kEmptyOutput;

    public LookaheadCalculator(
            Supplier<ChassisSpeeds> getFieldRelativeSpeeds,
            Supplier<Rotation2d> getRobotYaw,
            Supplier<RawAimingParameters> getLookaheadInput) {
        this(getFieldRelativeSpeeds, getRobotYaw, getLookaheadInput, () -> false, 0.0);
    }

    public LookaheadCalculator(
            Supplier<ChassisSpeeds> getFieldRelativeSpeeds,
            Supplier<Rotation2d> getRobotYaw,
            Supplier<RawAimingParameters> getLookaheadInput,
            BooleanSupplier allowShootOnTheMove,
            double shotTime) {
        this.getFieldRelativeSpeeds = getFieldRelativeSpeeds;
        this.getRobotYaw = getRobotYaw;
        this.getLookaheadInput = getLookaheadInput;
        this.allowShootOnTheMove = allowShootOnTheMove;
        this.shotTime = shotTime;
    }

    public void periodic() {
        final var input = this.getLookaheadInput.get();
        final ChassisSpeeds velocity = getFieldRelativeSpeeds.get();
        final double robotYaw = getRobotYaw.get().getDegrees();
        final double robotToGoalRotation = input.yawToTarget() + kOneEighty.getDegrees();
        final double rawYawSetpoint = robotYaw + robotToGoalRotation;
        double effectiveYawSetpoint = rawYawSetpoint;

        final double rawRangeToTarget = input.rangeToTargetMeters();
        double effectiveRangeToTarget = rawRangeToTarget;

        final Translation2d velocityTranslational =
                new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond)
                        .rotateBy(Rotation2d.fromDegrees(robotToGoalRotation));
        final double tangentialComponent = velocityTranslational.getY();
        final double radialComponent = velocityTranslational.getX();

        if (allowShootOnTheMove.getAsBoolean()) {
            double shotSpeed = Math.max(0, rawRangeToTarget / shotTime - radialComponent);
            double yawAdjustment =
                    Units.radiansToDegrees(Math.atan2(-tangentialComponent, shotSpeed));
            effectiveYawSetpoint += yawAdjustment;
            effectiveRangeToTarget =
                    shotTime
                            * Math.sqrt(
                                    PoseUtil.square(tangentialComponent)
                                            + PoseUtil.square(shotSpeed));
        }
        double effectiveYawFeedforwardVelocityDegPS =
                -Units.radiansToDegrees(tangentialComponent / rawRangeToTarget);
        double effectiveRangeFeedforwardMpS = -radialComponent * 0.1;
        this.log("Yaw feedforward", effectiveYawFeedforwardVelocityDegPS);
        this.log("Yaw setpoint", effectiveYawSetpoint);
        this.latestOutput =
                new LookaheadAimingParameters(
                        effectiveYawSetpoint,
                        effectiveRangeToTarget,
                        effectiveYawFeedforwardVelocityDegPS,
                        effectiveRangeFeedforwardMpS,
                        rawYawSetpoint,
                        rawRangeToTarget);
    }

    public LookaheadAimingParameters getLatestAimingParameters() {
        return this.latestOutput;
    }

    public double getEffectiveRangeWithFF() {
        return this.latestOutput.effectiveRangeToTarget
                + this.latestOutput.effectiveRangeFeedforwardMpS;
    }

    public double getEffectiveRangeToTarget() {
        return this.latestOutput.effectiveRangeToTarget;
    }

    public double getRawRangeToTarget() {
        return this.latestOutput.rawRangeToTarget;
    }
}

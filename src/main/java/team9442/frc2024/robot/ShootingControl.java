package team9442.frc2024.robot;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import team9442.frc2024.constants.ShooterLookupTable;
import team9442.frc2024.util.SpeakerScoreType;
import team9442.lib.VirtualSubsystem;

public class ShootingControl extends VirtualSubsystem implements Logged {

    public SpeakerScoreType scoreType = SpeakerScoreType.kFender;
    private double aimedSurfaceVelocity = 0.0;
    private double aimedPivotAngle = 0.0;
    private double aimedDistance = 0;

    private double angleAdjust = 0.0;
    private double velocityAdjust = 0.0;

    private final DoubleSupplier getDistanceToTarget;

    public ShootingControl(DoubleSupplier getDistanceToTarget) {
        this.getDistanceToTarget = getDistanceToTarget;
        this.setSpeakerScore(SpeakerScoreType.kAuto);
    }

    public void periodic() {
        aimedDistance = this.getDistanceToTarget.getAsDouble();
        if (scoreType.isStaticShot()) {
            aimedSurfaceVelocity = scoreType.getVelocity();
            aimedPivotAngle = scoreType.getAngle();
        } else {
            // potentially use lookahead time
            aimedSurfaceVelocity = ShooterLookupTable.getVelocityForDistance(aimedDistance);
            aimedPivotAngle = ShooterLookupTable.getAngleForDistance(aimedDistance);
        }

        aimedSurfaceVelocity += velocityAdjust;
        aimedPivotAngle += angleAdjust;

        this.log("Shooter Angle Setpoint", this.getShooterAngle());
        this.log("Shooter Velocity Setpoint", this.getSpeakerVelocity());
        this.log("manual override", this.scoreType.isStaticShot());
        this.log("angle adjustment", this.angleAdjust);
    }

    public double getSpeakerVelocity() {
        return this.aimedSurfaceVelocity;
    }

    public double getShooterAngle() {
        return this.aimedPivotAngle;
    }

    public double getAimedDistance() {
        return this.aimedDistance;
    }

    public Command increaseAngleAdjustment() {
        return runOnce(() -> angleAdjust += 0.25);
    }

    public Command decreaseAngleAdjustment() {
        return runOnce(() -> angleAdjust -= 0.25);
    }

    public Command increaseVelocityAdjustment() {
        return runOnce(() -> velocityAdjust += 0.5);
    }

    public Command decreaseVelocityAdjustment() {
        return runOnce(() -> velocityAdjust -= 0.5);
    }

    public Command setAutoScoring() {
        return runOnce(() -> setSpeakerScore(SpeakerScoreType.kAuto));
    }

    public Command setFarTrussScoring() {
        return runOnce(() -> setSpeakerScore(SpeakerScoreType.kFarTruss));
    }

    public Command setFenderScoring() {
        return runOnce(() -> setSpeakerScore(SpeakerScoreType.kFender));
    }

    public Command setProtectedScoring() {
        return runOnce(() -> setSpeakerScore(SpeakerScoreType.kProtected));
    }

    public Command setSpeakerScoreC(SpeakerScoreType speakerScoreType) {
        return runOnce(() -> this.setSpeakerScore(speakerScoreType));
    }

    public void setSpeakerScore(SpeakerScoreType speakerScoreType) {
        this.scoreType = speakerScoreType;
    }

    public boolean isManualOverride() {
        return this.scoreType.isStaticShot();
    }

    public double getAngleAdjust() {
        return angleAdjust;
    }
}

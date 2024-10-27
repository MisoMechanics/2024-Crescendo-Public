package team9442.frc2024.util;

import team9442.frc2024.constants.ShooterPivotConstants;
import team9442.frc2024.constants.ShooterRollersConstants;

public class SpeakerScoreType {
    public static final SpeakerScoreType kAuto = new SpeakerScoreType(0, 0, false);
    public static final SpeakerScoreType kFender =
            new SpeakerScoreType(
                    ShooterRollersConstants.kSpeakerScoreClosedLoopFender,
                    ShooterPivotConstants.kSpeakerFenderScoreAngle,
                    true);
    public static final SpeakerScoreType kFenderAuto =
            new SpeakerScoreType(ShooterRollersConstants.kSpeakerScoreClosedLoopFender, 58, true);
    public static final SpeakerScoreType kProtected =
            new SpeakerScoreType(
                    ShooterRollersConstants.kSpeakerScoreClosedLoopProtected,
                    ShooterPivotConstants.kSpeakerPodiumScoreAngle,
                    true);
    public static final SpeakerScoreType kProtectedAuto =
            new SpeakerScoreType(
                    ShooterRollersConstants.kSpeakerScoreClosedLoopProtected, 37, true);
    public static final SpeakerScoreType kFarTruss =
            new SpeakerScoreType(
                    ShooterRollersConstants.kSpeakerScoreClosedLoopFarTruss,
                    ShooterPivotConstants.kSpeakerFarTrussScoreAngle,
                    true);

    private double velocity;
    private double angle;
    private final boolean staticShot;

    public double getVelocity() {
        return velocity;
    }

    public double getAngle() {
        return angle;
    }

    public boolean isStaticShot() {
        return staticShot;
    }

    public void adjustVelocity(double adjustment) {
        this.velocity += adjustment;
    }

    public void adjustAngle(double adjustment) {
        this.angle += adjustment;
    }

    public SpeakerScoreType(double velocity, double angle, boolean staticShot) {
        this.velocity = velocity;
        this.angle = angle;
        this.staticShot = staticShot;
    }
}

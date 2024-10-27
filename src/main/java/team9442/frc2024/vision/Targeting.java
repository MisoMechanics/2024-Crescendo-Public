package team9442.frc2024.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;
import team9442.frc2024.vision.LookaheadCalculator.RawAimingParameters;
import team9442.lib.AllianceUpdatedObserver;
import team9442.lib.VirtualSubsystem;

public class Targeting extends VirtualSubsystem implements AllianceUpdatedObserver {
    private final Supplier<Pose2d> getDrivePose;
    private final Pose2d blueCenterTag;
    private final Pose2d redCenterTag;
    private Transform2d offsetTransform = new Transform2d();

    private Pose2d fieldToCenterTag = new Pose2d();
    private Pose2d fieldToSpeakerWithOffset = new Pose2d();

    private static final RawAimingParameters kEmptyParams = new RawAimingParameters(0, 0);
    private RawAimingParameters latestRawAimingParameters = kEmptyParams;

    public Targeting(Supplier<Pose2d> getDrivePose, Pose2d blueCenterTag, Pose2d redCenterTag) {
        this(getDrivePose, blueCenterTag, redCenterTag, 0);
    }

    public Targeting(
            Supplier<Pose2d> getDrivePose,
            Pose2d blueCenterTag,
            Pose2d redCenterTag,
            double speakerOffset) {
        this.getDrivePose = getDrivePose;
        this.blueCenterTag = blueCenterTag;
        this.redCenterTag = redCenterTag;
        this.fieldToCenterTag = blueCenterTag;
        this.offsetTransform = new Transform2d(speakerOffset, 0, new Rotation2d());
        this.fieldToSpeakerWithOffset = this.fieldToCenterTag.transformBy(this.offsetTransform);
    }

    @Override
    public void periodic() {
        final var robotToCenterTag = this.fieldToCenterTag.minus(getDrivePose.get());
        final var robotToSpeakerWithOffset =
                this.fieldToSpeakerWithOffset.minus(getDrivePose.get());
        final var rangeMeters = robotToCenterTag.getTranslation().getNorm();
        final var angleToTarget = robotToSpeakerWithOffset.getTranslation().getAngle();
        this.latestRawAimingParameters =
                new RawAimingParameters(angleToTarget.getDegrees(), rangeMeters);
    }

    public void onAllianceFound(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            this.fieldToCenterTag = blueCenterTag;
        } else {
            this.fieldToCenterTag = redCenterTag;
        }
        this.fieldToSpeakerWithOffset = this.fieldToCenterTag.transformBy(offsetTransform);
    }

    public Pose2d getFieldToCenterTag() {
        return this.fieldToCenterTag;
    }

    public Pose2d getFieldToSpeakerWithOffset() {
        return this.fieldToSpeakerWithOffset;
    }

    public double getLatestDistance() {
        return this.latestRawAimingParameters.rangeToTargetMeters();
    }

    public double getLatestAngle() {
        return this.latestRawAimingParameters.yawToTarget();
    }

    public RawAimingParameters getLatestParameters() {
        return this.latestRawAimingParameters;
    }
}

package team9442.frc2024.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import team9442.frc2024.constants.FieldConstants;
import team9442.frc2024.constants.VisionConstants;
import team9442.lib.VirtualSubsystem;

public class Vision extends VirtualSubsystem implements Logged {
    public static record PoseEstimate(
            EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

    public static record EstimateAndInfo(
            EstimatedRobotPose visionEstimate, String cameraName, double ambiguity) {}

    public static record CamToEstimator(PhotonCamera photonCamera, PhotonPoseEstimator estimator) {}

    private final List<CamToEstimator> cameras;
    private final Consumer<PoseEstimate> dtUpdateEstimate;

    public Vision(List<CamToEstimator> cameras, Consumer<PoseEstimate> dtUpdateEstimate) {
        this.cameras = cameras;
        this.dtUpdateEstimate = dtUpdateEstimate;
        for (final var camToEstimator : this.cameras) {
            camToEstimator.estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    public void periodic() {
        this.cameras.stream()
                .map(Vision::updateAngGetEstimate)
                .flatMap(Optional::stream)
                .map(this.logPose("%s - unfiltered pose3d"))
                .filter(Vision::isUsingTwoTags)
                .filter(Vision::zIsRight)
                .filter(Vision::isOnField)
                .filter(Vision::maxDistanceIsInThreshold)
                .filter(Vision.isAmbiguityLess(0.25))
                .filter(Vision::pitchRollAreInBounds)
                .map(this.logPose("%s - filtered pose3d"))
                .map(Vision::generatePoseEstimate)
                .forEach(dtUpdateEstimate);
    }

    /**
     * Returns the poses of all currently visible tags.
     *
     * @return An array of Pose3ds.
     */
    @Log.NT
    public Pose3d[] getSeenTags() {
        return cameras.stream()
                .flatMap(c -> c.photonCamera().getLatestResult().targets.stream())
                .map(PhotonTrackedTarget::getFiducialId)
                .map(VisionConstants.kTagLayout::getTagPose)
                .map(Optional::get)
                .toArray(Pose3d[]::new);
    }

    private static PoseEstimate generatePoseEstimate(EstimateAndInfo estimateAndInfo) {
        double maxDistance =
                estimateAndInfo.visionEstimate.targetsUsed.stream()
                        .mapToDouble(
                                target -> target.getBestCameraToTarget().getTranslation().getNorm())
                        .max()
                        .orElse(0.0);

        final var stdDevs =
                VisionConstants.kMultiTagStdDevs
                        .times(maxDistance)
                        .times(4 / Math.pow(estimateAndInfo.visionEstimate.targetsUsed.size(), 2));
        return new PoseEstimate(estimateAndInfo.visionEstimate, stdDevs);
    }

    private static Optional<EstimateAndInfo> updateAngGetEstimate(CamToEstimator camToEstimator) {
        final var latestResult = camToEstimator.photonCamera.getLatestResult();
        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }
        final var estimatedPose = camToEstimator.estimator.update(latestResult);
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }
        final var ambiguity = latestResult.getBestTarget().getPoseAmbiguity();
        return Optional.of(
                new EstimateAndInfo(
                        estimatedPose.get(), camToEstimator.photonCamera().getName(), ambiguity));
    }

    public Function<EstimateAndInfo, EstimateAndInfo> logPose(String key) {
        return estimateAndInfo -> {
            this.log(
                    String.format(key, estimateAndInfo.cameraName),
                    estimateAndInfo.visionEstimate.estimatedPose);
            return estimateAndInfo;
        };
    }

    private static EstimateAndInfo getSmallestAmbiguity(EstimateAndInfo one, EstimateAndInfo two) {
        return one.ambiguity < two.ambiguity ? one : two;
    }

    private static Predicate<EstimateAndInfo> isAmbiguityLess(double maxAmbiguity) {
        return estimateAndInfo ->
                estimateAndInfo.visionEstimate.targetsUsed.stream()
                        .allMatch(trackedTarget -> trackedTarget.getPoseAmbiguity() < maxAmbiguity);
    }

    private static boolean isUsingTwoTags(EstimateAndInfo estimateAndInfo) {
        return estimateAndInfo.visionEstimate.targetsUsed.size() >= 2;
    }

    private static boolean maxDistanceIsInThreshold(EstimateAndInfo estimateAndInfo) {
        double maxDistance =
                estimateAndInfo.visionEstimate.targetsUsed.stream()
                        .mapToDouble(
                                target -> target.getBestCameraToTarget().getTranslation().getNorm())
                        .max()
                        .orElse(0.0);

        return 0.5 < maxDistance && maxDistance < 8.0;
    }

    private static boolean isOnField(EstimateAndInfo estimateAndInfo) {
        return FieldConstants.isOnField(estimateAndInfo.visionEstimate.estimatedPose);
    }

    private static boolean zIsRight(EstimateAndInfo estimateAndInfo) {
        return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getZ()) < 0.2;
    }

    private static boolean pitchRollAreInBounds(EstimateAndInfo estimateAndInfo) {
        return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getRotation().getX()) < 0.2
                && Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getRotation().getY())
                        < 0.2;
    }
}

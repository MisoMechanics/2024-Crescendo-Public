package team9442.frc2024.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public final class VisionConstants {
    public static final String kLeftCamName = "Port";
    public static final String kRightCamName = "Starboard";
    public static final String kCenterCamName = "Center";
    public static final String kNoteCamName = "note";

    public static final Transform3d kRobotToCamLeft =
            new Transform3d()
                    .plus(
                            new Transform3d(
                                    new Translation3d(
                                            Units.inchesToMeters(-8.875),
                                            Units.inchesToMeters(10.5),
                                            Units.inchesToMeters(8.25)),
                                    new Rotation3d()
                                            .rotateBy(
                                                    new Rotation3d(
                                                            0,
                                                            Units.degreesToRadians(-30.6),
                                                            Units.degreesToRadians(-30 + 180)))));

    public static final Transform3d kRobotToCamRight =
            new Transform3d()
                    .plus(
                            new Transform3d(
                                    new Translation3d(
                                            Units.inchesToMeters(-8.875),
                                            Units.inchesToMeters(-10.5),
                                            Units.inchesToMeters(8.25)),
                                    new Rotation3d()
                                            .rotateBy(
                                                    new Rotation3d(
                                                            0,
                                                            Units.degreesToRadians(-30.6),
                                                            Units.degreesToRadians(30 + 180)))));

    public static final Transform3d kRobotToCamCenter =
            new Transform3d()
                    .plus(
                            new Transform3d(
                                    new Translation3d(
                                            Units.inchesToMeters(-4.154),
                                            Units.inchesToMeters(0),
                                            Units.inchesToMeters(9.804)),
                                    new Rotation3d()
                                            .rotateBy(
                                                    new Rotation3d(
                                                            0,
                                                            Units.degreesToRadians(-30),
                                                            Units.degreesToRadians(180)))));

    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, Double.MAX_VALUE);
    public static final Matrix<N3, N1> kMultiTagStdDevs =
            VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY);

    public static final PhotonCamera noteCam = new PhotonCamera(kNoteCamName);
    //    public static final PhotonCamera portCam = new PhotonCamera(kLeftCamName);
    //    public static final PhotonPoseEstimator portEstimator =
    //            new PhotonPoseEstimator(
    //                    kTagLayout,
    //                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //                    portCam,
    //                    kRobotToCamLeft);

    //     public static final PhotonCamera starboardCam = new PhotonCamera(kRightCamName);
    //     public static final PhotonPoseEstimator starboardEstimator =
    //             new PhotonPoseEstimator(
    //                     kTagLayout,
    //                     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //                     starboardCam,
    //                     VisionConstants.kRobotToCamRight);

    public static final PhotonCamera centerCam = new PhotonCamera(kCenterCamName);
    public static final PhotonPoseEstimator centerEstimator =
            new PhotonPoseEstimator(
                    kTagLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    centerCam,
                    VisionConstants.kRobotToCamCenter);
}

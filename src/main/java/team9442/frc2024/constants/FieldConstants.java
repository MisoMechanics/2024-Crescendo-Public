package team9442.frc2024.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final double kFieldWidthMeters = 16.5410515;
    public static final double kHalfFieldMeters = kFieldWidthMeters / 2.0;
    public static final Rotation2d kPiRotation = new Rotation2d(Math.PI);

    public static Pose2d flipped(Pose2d pose) {
        return new Pose2d(
                new Translation2d(kFieldWidthMeters - pose.getX(), pose.getY()),
                kPiRotation.minus(pose.getRotation()));
    }

    public static boolean isOnField(Pose3d pose) {
        return pose.getX() >= 0.0
                && pose.getX() <= kFieldWidthMeters
                && pose.getY() >= 0.0
                && pose.getY() <= 8.229;
    }
}

package team9442.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class PoseUtil {

    public static double square(double x) {
        return Math.pow(x, 2);
    }

    public static double norm(Pose2d a, Pose2d b) {
        return Math.sqrt(square(a.getX() - b.getX()) + square(a.getY() - b.getY()));
    }

    public static double norm(Translation3d transform3d) {
        return Math.sqrt(
                square(transform3d.getX())
                        + square(transform3d.getY())
                        + square(transform3d.getZ()));
    }
}

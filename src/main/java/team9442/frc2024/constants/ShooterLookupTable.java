package team9442.frc2024.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class ShooterLookupTable {
    private static final InterpolatingDoubleTreeMap kVelocityLookupTable =
            new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap kAngleLookupTable =
            new InterpolatingDoubleTreeMap();

    public static double getVelocityForDistance(double distance) {
        return kVelocityLookupTable.get(distance);
    }

    public static double getAngleForDistance(double distance) {
        return kAngleLookupTable.get(distance);
    }

    public static final double kSpeakerOffsetMeters = Units.inchesToMeters(37);

    private static final double[][] kFlywheelMap = {
        {1.34, 24},
        {1.62, 24},
        {1.845, 24},
        {2.1, 24},
        {2.36, 24},
        {2.61, 24},
        {2.87, 24},
        {3.16, 24},
        {3.4, 24},
        {3.64, 24},
        {3.92, 24},
        {4.17, 24},
        {4.4, 24},
        {4.5, 24},
        {4.9, 24},
        {5.2, 24},
        {5.5, 27.125},
        {6.03, 28.25}
    };

    private static final double[][] kPivotMap = {
        {1.32, 55.75},
        {1.67, 49.625},
        {1.92, 45.25},
        {2.12, 43.125},
        {2.42, 39.25},
        {2.67, 37.125},
        {2.86, 34.91},
        {3.39, 31.75},
        {3.58, 28.825},
        {3.8, 28.5},
        {4.02, 28.53},
        {4.25, 27.7975},
        {4.66, 26.25},
        {4.85, 25.08},
        {4.94, 25},
        {5.2, 25},
        {5.5, 22.25},
        {6.03, 21.93}
    };

    static {
        for (double[] pair : kFlywheelMap) {
            kVelocityLookupTable.put(pair[0], pair[1]);
        }

        for (double[] pair : kPivotMap) {
            kAngleLookupTable.put(pair[0], pair[1]);
        }
    }
}

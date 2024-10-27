package team9442.frc2024.constants;

public final class GlobalConstants {
    /** loop time in seconds */
    public static final double kDt = 0.02;

    public static final String kCANbusName = "drive";

    public static final class IntakeIds {

        public static final int kIntakeId = 9;
        public static final int kIndexerId = 10;

        private IntakeIds() {}
    }

    public static final class FeederIds {

        public static final int kMasterId = 11;
        public static final int kFeederSensorPort = 0;
        public static final int kStagingSensorPort = 1;

        private FeederIds() {}
    }

    public static final class ShooterPivotIds {

        public static final int kMasterId = 12;

        private ShooterPivotIds() {}
    }

    public static final class ShooterRollersIds {

        public static final int kPortId = 13;
        public static final int kStarboardId = 14;

        private ShooterRollersIds() {}
    }

    public static final class AmpPivotIds {
        public static final int kMasterId = 15;

        private AmpPivotIds() {}
    }

    public static final class AmpRollersIds {
        public static final int kMasterId = 16;

        private AmpRollersIds() {}
    }

    public static final class TelescopeIds {
        public static final int KPortSideId = 17;
        public static final int kStarboardSideId = 18;

        private TelescopeIds() {}
    }

    private GlobalConstants() {}
}

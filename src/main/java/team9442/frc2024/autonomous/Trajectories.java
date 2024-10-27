package team9442.frc2024.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import team9442.frc2024.constants.VisionConstants;

public final class Trajectories {

    public static final Pose2d kBlueSideSpeaker =
            VisionConstants.kTagLayout.getTagPose(3).get().toPose2d();
    public static final ChoreoTrajectory straight = Choreo.getTrajectory("straight 4m");

    public static final class AmpSideClose {
        public static final ChoreoTrajectory AmpFender_CN1 = Choreo.getTrajectory("AmpFender-CN1");
        public static final ChoreoTrajectory CN1_CN2 = Choreo.getTrajectory("CN1-CN2");
        public static final ChoreoTrajectory CN2_CN3 = Choreo.getTrajectory("CN2-CN3");
        public static final ChoreoTrajectory CN3_MN5 = Choreo.getTrajectory("CN3-MN5");
        public static final ChoreoTrajectory MN5_SourceTruss =
                Choreo.getTrajectory("MN5-SourceTruss");
        public static final ChoreoTrajectory CN3_Garbage = Choreo.getTrajectory("CN3-Garbage");
        public static final ChoreoTrajectory Garbage_SourceTruss =
                Choreo.getTrajectory("Garbage-SourceTruss");
        public static final ChoreoTrajectory SourceTruss_GarbageTwo =
                Choreo.getTrajectory("SourceTruss-GarbageTwo");
    }

    public static final class AmpSideFar {
        public static final ChoreoTrajectory Amp_MN1 = Choreo.getTrajectory("Amp-MN1");
        public static final ChoreoTrajectory MN1_AmpTruss = Choreo.getTrajectory("MN1-AmpTruss");
        public static final ChoreoTrajectory AmpTruss_MN2 = Choreo.getTrajectory("AmpTruss-MN2");
        public static final ChoreoTrajectory MN2_AmpTruss = Choreo.getTrajectory("MN2-AmpTruss");
        public static final ChoreoTrajectory AmpTruss_MN3 = Choreo.getTrajectory("AmpTruss-MN3");
        public static final ChoreoTrajectory MN3_AmpTruss = Choreo.getTrajectory("MN3-AmpTruss");
        public static final ChoreoTrajectory Dynamic_AmpTruss =
                Choreo.getTrajectory("Dynamic-AmpTruss");
        public static final ChoreoTrajectory MN1_MN4 = Choreo.getTrajectory("MN1-MN4");
        public static final ChoreoTrajectory MN2_MN4 = Choreo.getTrajectory("MN2-MN4");
    }

    public static final class SourceSideClose {
        public static final ChoreoTrajectory SourceFender_CN3 =
                Choreo.getTrajectory("SourceFender-CN3");
        public static final ChoreoTrajectory CN3_CN2 = Choreo.getTrajectory("CN3-CN2");
        public static final ChoreoTrajectory CN2_CN1 = Choreo.getTrajectory("CN2-CN1");
        public static final ChoreoTrajectory CN1_MN1 = Choreo.getTrajectory("CN1-MN1");
        public static final ChoreoTrajectory MN1_AmpTruss = Choreo.getTrajectory("MN1-AmpTruss");
        public static final ChoreoTrajectory AmpTruss_MN2 = Choreo.getTrajectory("AmpTruss-MN2");
        public static final ChoreoTrajectory MN2_AmpTruss = Choreo.getTrajectory("MN2-AmpTruss");
        public static final ChoreoTrajectory Center_CN1 = Choreo.getTrajectory("Center-CN1");
        public static final ChoreoTrajectory CN1_Garbage = Choreo.getTrajectory("CN1-Garbage");
        public static final ChoreoTrajectory Garbage_Score = Choreo.getTrajectory("Garbage-Score");
        public static final ChoreoTrajectory GarbageScore_GarbageTwo =
                Choreo.getTrajectory("GarbageScore-GarbageTwo");
    }

    public static final class SourceSideFar {
        public static final ChoreoTrajectory sourceFender_MN5 =
                Choreo.getTrajectory("SourceFender-MN5");
        public static final ChoreoTrajectory MN5_Center = Choreo.getTrajectory("MN5-Center");
        public static final ChoreoTrajectory Center_MN4 = Choreo.getTrajectory("Center-MN4");
        public static final ChoreoTrajectory MN4_Center = Choreo.getTrajectory("MN4-Center");
        public static final ChoreoTrajectory Center_MN3 = Choreo.getTrajectory("Center-MN3");
        public static final ChoreoTrajectory MN3_Center = Choreo.getTrajectory("MN3-Center");
        public static final ChoreoTrajectory MN5_MN2 = Choreo.getTrajectory("MN5-MN2");
        public static final ChoreoTrajectory MN4_MN2 = Choreo.getTrajectory("MN4-MN2");
        public static final ChoreoTrajectory Dynamic_SourceTruss =
                Choreo.getTrajectory("Dynamic-SourceTruss");
        public static final ChoreoTrajectory SourceTruss_MN4 =
                Choreo.getTrajectory("SourceTruss-MN4");
    }

    public static final class SourceSideFarFlip {
        public static final ChoreoTrajectory sourceFender_MN4B =
                Choreo.getTrajectory("SourceFender-MN4B");
        public static final ChoreoTrajectory MN4B_SourceTruss =
                Choreo.getTrajectory("MN4B-SourceTruss");
        public static final ChoreoTrajectory SourceTruss_MN5B =
                Choreo.getTrajectory("SourceTruss-MN5B");
        public static final ChoreoTrajectory MN5B_SourceTruss =
                Choreo.getTrajectory("MN5B-SourceTruss");
        public static final ChoreoTrajectory SourceTruss_MN3 =
                Choreo.getTrajectory("SourceTruss-MN3");
        public static final ChoreoTrajectory MN3_SourceTruss =
                Choreo.getTrajectory("MN3-SourceTruss");
        public static final ChoreoTrajectory MN4B_MN5B = Choreo.getTrajectory("MN4B-MN5B");
        public static final ChoreoTrajectory MN5B_MN3 = Choreo.getTrajectory("MN5B-MN3");
    }

    public static final class BreadMetal {
        public static final ChoreoTrajectory Source_MN5 = Choreo.getTrajectory("Source-MN5");
    }

    public static final class BearTrap {
        public static final ChoreoTrajectory beartrap1 = Choreo.getTrajectory("beartrap1");
        public static final ChoreoTrajectory beartrap2 = Choreo.getTrajectory("beartrap2");
        public static final ChoreoTrajectory beartrap3 = Choreo.getTrajectory("beartrap3");
        public static final ChoreoTrajectory beartrap4 = Choreo.getTrajectory("beartrap4");
    }

    public static final class troll {
        public static final ChoreoTrajectory troll1 = Choreo.getTrajectory("troll1");
        public static final ChoreoTrajectory troll2 = Choreo.getTrajectory("troll2");
    }
}

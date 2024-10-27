package team9442.frc2024.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class DrivetrainConstants {

    public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward(0, 0);

    public static final double kDriveMaxSpeedMetersPerSecond = 4.5;

    private static final double kWheelDiameterMeters = 0.1016;
    private static final double kDriveGearboxReduction = 1 / 6.75;

    public static final double kTurnEncoderToDegrees = 1;

    public static final double kWheelRotationToMetersDrive =
            kDriveGearboxReduction * kWheelDiameterMeters * Math.PI;

    public static final ProfiledPIDController kAmpAimYawControllers =
            new ProfiledPIDController(12, 0, 1.2, new TrapezoidProfile.Constraints(458, 916));
    public static final PIDController kNoteYawController = new PIDController(5, 0, 0);
    public static final PIDController kNoteXController = new PIDController(0.1, 0, 0);
    public static final PIDController kNoteYController = new PIDController(0.1, 0, 0);
    public static final ProfiledPIDController kRotControllerDegrees =
            new ProfiledPIDController(12, 0, 1.2, new TrapezoidProfile.Constraints(458, 916));

    static {
        kAmpAimYawControllers.enableContinuousInput(-180, 180);
        kRotControllerDegrees.enableContinuousInput(-180, 180);
        kRotControllerDegrees.setTolerance(2, 3);
        kAmpAimYawControllers.setTolerance(2, 3);
    }

    private DrivetrainConstants() {}
}

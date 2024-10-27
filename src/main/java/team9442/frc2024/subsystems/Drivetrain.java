package team9442.frc2024.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Arrays;
import monologue.Logged;
import team9442.frc2024.constants.FieldConstants;
import team9442.frc2024.vision.Vision.PoseEstimate;
import team9442.lib.AllianceUpdatedObserver;
import team9442.lib.CustomSwerveRequests;
import team9442.lib.PeriodicSubsystem;
import team9442.lib.PoseUtil;

public class Drivetrain extends SwerveDrivetrain
        implements PeriodicSubsystem, AllianceUpdatedObserver, Logged {

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private static final Rotation2d kZero = Rotation2d.fromDegrees(0);
    private static final Rotation2d kOneEighty = Rotation2d.fromDegrees(180);

    private double pitchZero = 0;

    private double cachedSpeed = 0;

    private final SysIdRoutine driveSysIdRoutine;
    private final SysIdRoutine steerSysIdRoutine;

    private boolean resetToVision = false;

    public static class PeriodicIO {
        // inputs

        public double characterizationVoltage = 0;
        public boolean isOpenloop = true;
        public double heading = 0;
        public double roll = 0;
        public double pitch = 0;
        public double rawHeading = 0;
        public Rotation2d gyroRotation = new Rotation2d();

        public SwerveModuleState frontLeftState = new SwerveModuleState();
        public SwerveModuleState frontRightState = new SwerveModuleState();
        public SwerveModuleState backLeftState = new SwerveModuleState();
        public SwerveModuleState backRightState = new SwerveModuleState();

        public double timestamp = 0;

        public Pose2d visionPose = new Pose2d();

        public SwerveRequest masterRequest = new SwerveRequest.Idle();
        public CustomSwerveRequests.CharacterizeDrive characterizeDriveFOCRequest =
                new CustomSwerveRequests.CharacterizeDrive();
        public CustomSwerveRequests.CharacterizeTurn characterizeSteerFOCRequest =
                new CustomSwerveRequests.CharacterizeTurn();
    }

    public Drivetrain(
            SwerveDrivetrainConstants swerveDriveConstants,
            double maxSpeedMpS,
            double maxRotRadPerSec,
            SwerveModuleConstants... swerveModuleConstants) {
        super(swerveDriveConstants, swerveModuleConstants);
        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        TorqueCurrentConfigs torqueLimit = new TorqueCurrentConfigs();

        torqueLimit.PeakForwardTorqueCurrent = 60;
        torqueLimit.PeakReverseTorqueCurrent = -60;

        for (final var module : this.Modules) {
            module.getDriveMotor().getConfigurator().apply(torqueLimit, 1);
        }
        this.driveSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.of(10).per(Units.Seconds.of(1)),
                                Units.Volts.of(60),
                                Units.Seconds.of(6),
                                state -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts ->
                                        this.setRequest(
                                                periodicIO.characterizeDriveFOCRequest
                                                        .withTargetTorque(volts.in(Units.Volts))),
                                null,
                                this));

        this.steerSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.of(4).per(Units.Seconds.of(1)),
                                Units.Volts.of(20),
                                Units.Seconds.of(6),
                                state -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts ->
                                        this.setRequest(
                                                periodicIO.characterizeSteerFOCRequest
                                                        .withTargetTorque(volts.in(Units.Volts))),
                                null,
                                this));
    }

    public void zeroPitch() {
        this.pitchZero = this.getPitch();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.roll = this.m_pigeon2.getRoll().getValue();
        periodicIO.heading = this.m_pigeon2.getYaw().getValue();
        periodicIO.pitch = this.m_pigeon2.getPitch().getValue() - this.pitchZero;
        periodicIO.rawHeading = this.m_pigeon2.getYaw().getValue();
        periodicIO.frontLeftState = this.Modules[0].getCurrentState();
        periodicIO.frontRightState = this.Modules[1].getCurrentState();
        periodicIO.backLeftState = this.Modules[2].getCurrentState();
        periodicIO.backRightState = this.Modules[3].getCurrentState();
        periodicIO.gyroRotation = Rotation2d.fromDegrees(periodicIO.heading);
        periodicIO.timestamp = Timer.getFPGATimestamp();

        this.log("raw heading", periodicIO.rawHeading);
        this.log("heading degrees", periodicIO.gyroRotation.getDegrees());
        this.log("frontLeftState", periodicIO.frontLeftState);
        this.log("frontRightState", periodicIO.frontRightState);
        this.log("backLeftState", periodicIO.backLeftState);
        this.log("backRightState", periodicIO.backRightState);
        this.log("estimated pose", this.m_odometry.getEstimatedPosition());
        this.log("acceleration  x", getAccel());
    }

    @Override
    public void writePeriodicOutputs() {
        this.setControl(periodicIO.masterRequest);
    }

    public void setRequest(SwerveRequest request) {
        this.periodicIO.masterRequest = request;
    }

    public double getRobotVx() {
        return this.getChassisSpeeds().vxMetersPerSecond;
    }

    public double getRobotVy() {
        return this.getChassisSpeeds().vyMetersPerSecond;
    }

    public double getNormalizedDtX() {
        return Math.abs(
                Math.abs(this.getOdoPose().getX() - FieldConstants.kHalfFieldMeters)
                        - FieldConstants.kHalfFieldMeters);
    }

    public Command runDriveQuasiTest(Direction direction) {
        return driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction) {
        return steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return steerSysIdRoutine.dynamic(direction);
    }

    public void setRobotPose(Pose2d pose) {
        seedFieldRelative(pose);
        periodicIO = new PeriodicIO();
    }

    @Override
    public void onAllianceFound(Alliance alliance) {
        switch (alliance) {
            case Red:
                this.setOperatorPerspectiveForward(kOneEighty);
                break;
            case Blue:
            default:
                this.setOperatorPerspectiveForward(kZero);
                break;
        }
    }

    public void resetEncoders() {
        for (final var module : this.Modules) {
            module.getCANcoder().setPosition(0);
        }
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public void setToBrake() {
        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
        for (final var module : this.Modules) {
            module.getDriveMotor().getConfigurator().refresh(outputConfig);
            outputConfig.NeutralMode = NeutralModeValue.Brake;
            module.getDriveMotor().getConfigurator().apply(outputConfig);
        }
    }

    public void zeroGyro() {
        this.m_pigeon2.setYaw(0.0);
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    public double getRoll() {
        return periodicIO.roll;
    }

    public double getPitch() {
        return periodicIO.pitch;
    }

    public double getRawHeading() {
        return periodicIO.rawHeading;
    }

    public Pose2d getOdoPose() {
        return this.m_odometry.getEstimatedPosition();
    }

    public Rotation2d getOdoRot() {
        return getOdoPose().getRotation();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.Modules[0].getPosition(true),
            this.Modules[1].getPosition(true),
            this.Modules[2].getPosition(true),
            this.Modules[3].getPosition(true)
        };
    }

    /** Get the position of all drive wheels in radians. */
    public double[] getWheelRadiusCharacterizationPosition() {
        return Arrays.stream(this.Modules)
                .mapToDouble(
                        module ->
                                edu.wpi.first.math.util.Units.rotationsToRadians(
                                        module.getDriveMotor().getPosition().getValueAsDouble()))
                .toArray();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public double getAccel() {
        double accel = (getChassisSpeeds().vxMetersPerSecond - cachedSpeed) / 0.02;
        this.cachedSpeed = getChassisSpeeds().vxMetersPerSecond;
        return accel;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getOdoRot());
    }

    public void updateEstimates(PoseEstimate poseEstimate) {
        final var visionEstimated = poseEstimate.estimatedPose().estimatedPose.toPose2d();
        this.log("robot pose3d", poseEstimate.estimatedPose().estimatedPose);

        // Multiply by 0.05 to converge to vision pose faster
        final var stddevs =
                resetToVision ? poseEstimate.standardDev().times(0.01) : poseEstimate.standardDev();

        this.addVisionMeasurement(
                visionEstimated, poseEstimate.estimatedPose().timestampSeconds, stddevs);
    }

    public void enableResetToVision() {
        this.resetToVision = true;
    }

    public void disableResetToVision() {
        this.resetToVision = false;
    }

    @Override
    public void end() {
        stopModules();
    }

    public Command stopModules() {
        return runOnce(() -> periodicIO.masterRequest = new SwerveRequest.Idle());
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            periodicIO.frontLeftState,
            periodicIO.frontRightState,
            periodicIO.backLeftState,
            periodicIO.backRightState
        };
    }

    public double getVel() {
        final var chassisSpeeds = this.getChassisSpeeds();
        return PoseUtil.square(chassisSpeeds.vxMetersPerSecond)
                + PoseUtil.square(chassisSpeeds.vyMetersPerSecond);
    }

    public double getMaxSpeedMpS() {
        return this.maxSpeedMpS;
    }

    public double getMaxRotationRadpS() {
        return this.maxRotRadPerSec;
    }
}

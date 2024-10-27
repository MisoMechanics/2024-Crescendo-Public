package team9442.frc2024.autonomous;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import monologue.LogLevel;
import monologue.Logged;
import team9442.frc2024.alignment.AutoAim;
import team9442.frc2024.alignment.NoteTracking;
import team9442.frc2024.robot.ShootingControl;
import team9442.frc2024.subsystems.Drivetrain;
import team9442.frc2024.subsystems.Superstructure;
import team9442.lib.AllianceUpdatedObserver;

public final class AutoCommands extends SubsystemBase
        implements AllianceUpdatedObserver, Iterable<AutoMode>, Logged {
    private final Superstructure superstructure;
    private final Drivetrain drivetrain;
    private final AutoAim autoAim;
    private final ShootingControl shootingControl;
    private final NoteTracking noteTracking;
    private final Trigger dtInShootingZone;
    private final Trigger dtStopped;
    private final Trigger dtInIntakeZone;
    private final double initialShot = 0.8;
    private final double shotDelay = 0.6;
    private final double rushShotDelay = 1;

    private Alliance alliance = Alliance.Blue;
    private final List<AutoMode> modes = new ArrayList<>();

    public final double kDtFarIntakeZone = 7;
    public final double kDtCloseIntakeZone = 2;

    private double dtIntakeZone = kDtFarIntakeZone;

    public AutoCommands(
            Drivetrain drivetrain,
            Superstructure superstructure,
            NoteTracking noteTracking,
            AutoAim autoAim,
            ShootingControl shootingControl) {
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;
        this.noteTracking = noteTracking;
        this.autoAim = autoAim;
        this.shootingControl = shootingControl;

        dtInShootingZone = new Trigger(() -> drivetrain.getNormalizedDtX() < 5);
        dtStopped = new Trigger(() -> drivetrain.getVel() < 0.3).debounce(0.5);
        dtInIntakeZone = new Trigger(() -> drivetrain.getNormalizedDtX() > dtIntakeZone);

        modes.add(
                new AutoMode(
                        "Amp Side Close",
                        getAmpSideClose(),
                        Trajectories.AmpSideClose.AmpFender_CN1.getInitialPose(),
                        this::setCloseIntakeZone));

        modes.add(
                new AutoMode(
                        "Amp Side Far",
                        getAmpSideFar(),
                        Trajectories.AmpSideFar.Amp_MN1.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Amp Side Far Rush",
                        getAmpSideFarRush(),
                        Trajectories.AmpSideFar.Amp_MN1.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Amp Side Far Dynamic",
                        getAmpSideFarDynamic(),
                        Trajectories.AmpSideFar.Amp_MN1.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Amp Side Garbage Collector",
                        getAmpSideGarbageCollection(),
                        Trajectories.AmpSideClose.AmpFender_CN1.getInitialPose(),
                        this::setCloseIntakeZone));

        modes.add(
                new AutoMode(
                        "Amp Side Two Note Garbage Collector",
                        getAmpSideTwoNoteGarbageCollection(),
                        Trajectories.AmpSideClose.AmpFender_CN1.getInitialPose(),
                        this::setCloseIntakeZone));

        modes.add(
                new AutoMode(
                        "Amp Side Troll",
                        getAmpSideTroll(),
                        Trajectories.troll.troll1.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Close",
                        getSourceSideClose(),
                        Trajectories.SourceSideClose.SourceFender_CN3.getInitialPose(),
                        this::setCloseIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Far",
                        getSourceSideFar(),
                        Trajectories.SourceSideFar.sourceFender_MN5.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Far Rush",
                        getSourceSideFarRush(),
                        Trajectories.BreadMetal.Source_MN5.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Far Dynamic",
                        getSourceSideFarDynamic(),
                        Trajectories.SourceSideFar.sourceFender_MN5.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Far Flip",
                        getSourceSideFarFlip(),
                        Trajectories.SourceSideFarFlip.sourceFender_MN4B.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Far MN2&3",
                        getSourceSideFarFlipMN23(),
                        Trajectories.SourceSideFarFlip.sourceFender_MN4B.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Far Flip Rush",
                        getSourceSideFarFlipRush(),
                        Trajectories.SourceSideFarFlip.sourceFender_MN4B.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Garbage Collector",
                        getSourceSideGarbageCollection(),
                        Trajectories.AmpSideClose.AmpFender_CN1.getInitialPose(),
                        this::setCloseIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Two Note Garbage Collector",
                        getSourceSideTwoNoteGarbageCollection(),
                        Trajectories.AmpSideClose.AmpFender_CN1.getInitialPose(),
                        this::setCloseIntakeZone));

        modes.add(
                new AutoMode(
                        "Source Side Troll",
                        getSourceSideTroll(),
                        Trajectories.troll.troll2.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "Bread with Metal",
                        getBreadMetal(),
                        Trajectories.BreadMetal.Source_MN5.getInitialPose(),
                        this::setFarIntakeZone));

        modes.add(
                new AutoMode(
                        "beartrap",
                        getBearTrap(),
                        Trajectories.BearTrap.beartrap1.getInitialPose(),
                        this::setFarIntakeZone));

        this.log("superstructure state", "");
    }

    public void periodic() {
        this.log(
                "dt in shooting zone",
                dtInShootingZone.getAsBoolean(),
                LogLevel.OVERRIDE_FILE_ONLY);
        this.log("dt stopped", dtStopped.getAsBoolean(), LogLevel.OVERRIDE_FILE_ONLY);
        this.log("dt in intake zone", dtInIntakeZone.getAsBoolean(), LogLevel.OVERRIDE_FILE_ONLY);
        this.log("Intake zone threshold", this.dtIntakeZone, LogLevel.OVERRIDE_FILE_ONLY);
    }

    public void setFarIntakeZone() {
        this.setIntakeZoneThreshold(kDtFarIntakeZone);
    }

    public void setCloseIntakeZone() {
        this.setIntakeZoneThreshold(kDtCloseIntakeZone);
    }

    public void setIntakeZoneThreshold(double threshold) {
        this.dtIntakeZone = threshold;
    }

    public Command getStraight4m() {
        return followPath(Trajectories.straight);
    }

    // --- Amp Side Far ---
    public Command getAmpSideFar() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(getDrivetrainAmpSideFar(follower), getSuperstructureAmpSideFar(follower));
    }

    private Command getSuperstructureAmpSideFar(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainAmpSideFar(CustomFollower follower) {
        return sequence(
                aimDrivetrain(1),
                stopDrivetrain(initialShot)
                        .until(superstructure.feederSensor.negate().debounce(0.3)),
                //                drivetrainHeading(() -> shouldFlip() ? 180 : 0).withTimeout(0.9),
                follower.followPath(),
                followPath(Trajectories.AmpSideFar.Amp_MN1, follower),
                followPath(Trajectories.AmpSideFar.MN1_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 0.15),
                followPath(Trajectories.AmpSideFar.AmpTruss_MN2, follower),
                followPath(Trajectories.AmpSideFar.MN2_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 0.15),
                followPath(Trajectories.AmpSideFar.AmpTruss_MN3, follower),
                followPath(Trajectories.AmpSideFar.MN3_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Amp Side Far ---
    public Command getAmpSideFarRush() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainAmpSideFarRush(follower), getSuperstructureAmpSideFarRush(follower));
    }

    private Command getSuperstructureAmpSideFarRush(CustomFollower follower) {
        return sequence(
                getSuperstructureFirstCycleRush(follower),
                getSuperstructureOneCycle(follower).repeatedly());
    }

    private Command getDrivetrainAmpSideFarRush(CustomFollower follower) {
        return sequence(
                parallel(
                        //                        sequence(waitSeconds(rushShotDelay),
                        // superstructure.spitBack()),
                        followPath(Trajectories.AmpSideFar.Amp_MN1, follower)),
                followPath(Trajectories.AmpSideFar.MN1_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 0.5),
                followPath(Trajectories.AmpSideFar.AmpTruss_MN2, follower),
                followPath(Trajectories.AmpSideFar.MN2_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 0.5),
                followPath(Trajectories.AmpSideFar.AmpTruss_MN3, follower),
                followPath(Trajectories.AmpSideFar.MN3_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Amp Side Far Dynamic ---
    public Command getAmpSideFarDynamic() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainAmpSideFarDynamic(follower),
                getSuperstructureAmpSideFarDynamic(follower));
    }

    private Command getSuperstructureAmpSideFarDynamic(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainAmpSideFarDynamic(CustomFollower follower) {
        return sequence(
                aimDrivetrain(1),
                stopDrivetrain(initialShot)
                        .until(superstructure.feederSensor.negate().debounce(0.3)),
                //                drivetrainHeading(() -> shouldFlip() ? 180 : 0).withTimeout(0.9),
                follower.followPath(),
                sequence(
                                followPath(Trajectories.AmpSideFar.Amp_MN1, follower),
                                followPath(Trajectories.AmpSideFar.MN1_MN4, follower))
                        .until(superstructure.hasNote),
                followPath(Trajectories.AmpSideFar.Dynamic_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                sequence(
                                followPath(Trajectories.AmpSideFar.AmpTruss_MN2, follower),
                                followPath(Trajectories.AmpSideFar.MN2_MN4, follower))
                        .until(superstructure.hasNote),
                followPath(Trajectories.AmpSideFar.Dynamic_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Amp Side Close ---
    public Command getAmpSideClose() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainAmpSideClose(follower), getSuperstructureAmpSideClose(follower));
    }

    private Command getSuperstructureAmpSideClose(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainAmpSideClose(CustomFollower follower) {
        return sequence(
                aimDrivetrain(1),
                stopDrivetrain(initialShot),
                followPath(Trajectories.AmpSideClose.AmpFender_CN1, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.AmpSideClose.CN1_CN2, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.AmpSideClose.CN2_CN3, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Amp Side Garbage Collection ---
    public Command getAmpSideGarbageCollection() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getSuperstructureAmpSideGarbageCollection(follower),
                getDrivetrainAmpSideGarbageCollection(follower));
    }

    private Command getSuperstructureAmpSideGarbageCollection(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainAmpSideGarbageCollection(CustomFollower follower) {
        return sequence(
                aimDrivetrain(1),
                stopDrivetrain(initialShot),
                followPath(Trajectories.AmpSideClose.AmpFender_CN1, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.AmpSideClose.CN1_CN2, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.AmpSideClose.CN2_CN3, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                Commands.runOnce(() -> setIntakeZoneThreshold(0)),
                followPath(Trajectories.AmpSideClose.CN3_Garbage, follower),
                aimNote(5),
                followPath(Trajectories.AmpSideClose.Garbage_SourceTruss, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Amp Side Two Note Garbage Collection ---
    public Command getAmpSideTwoNoteGarbageCollection() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getSuperstructureAmpSideTwoNoteGarbageCollection(follower),
                getDrivetrainAmpSideTwoNoteGarbageCollection(follower));
    }

    private Command getSuperstructureAmpSideTwoNoteGarbageCollection(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainAmpSideTwoNoteGarbageCollection(CustomFollower follower) {
        return sequence(
                aimDrivetrain(1),
                stopDrivetrain(initialShot),
                followPath(Trajectories.AmpSideClose.AmpFender_CN1, follower),
                aimDrivetrain(1),
                stopDrivetrain(initialShot),
                followPath(Trajectories.AmpSideClose.AmpFender_CN1, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.AmpSideClose.CN1_CN2, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.AmpSideClose.CN2_CN3, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                Commands.runOnce(() -> setIntakeZoneThreshold(0)),
                followPath(Trajectories.AmpSideClose.CN3_Garbage, follower),
                aimNote(5),
                followPath(Trajectories.AmpSideClose.Garbage_SourceTruss, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.AmpSideClose.SourceTruss_GarbageTwo, follower),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Source Side Close ---
    public Command getSourceSideClose() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideClose(follower), getSuperstructureSourceSideClose(follower));
    }

    private Command getSuperstructureSourceSideClose(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideClose(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideClose.SourceFender_CN3, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.CN3_CN2, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.CN2_CN1, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.CN1_MN1, follower),
                followPath(Trajectories.SourceSideClose.MN1_AmpTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Source Side Far ---
    public Command getSourceSideFar() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideFar(follower), getSuperstructureSourceSideFar(follower));
    }

    private Command getSuperstructureSourceSideFar(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideFar(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideFar.sourceFender_MN5, follower),
                followPath(Trajectories.SourceSideFar.MN5_Center, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFar.Center_MN4, follower),
                followPath(Trajectories.SourceSideFar.MN4_Center, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFar.Center_MN3, follower),
                stopDrivetrain(15));
    }
    // --- --- ---

    // --- Source Side Far ---
    public Command getSourceSideFarRush() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideFarRush(follower),
                getSuperstructureSourceSideFarRush(follower));
    }

    private Command getSuperstructureSourceSideFarRush(CustomFollower follower) {
        return sequence(
                getSuperstructureFirstCycleRush(follower),
                getSuperstructureOneCycle(follower).repeatedly());
    }

    private Command getDrivetrainSourceSideFarRush(CustomFollower follower) {
        return sequence(
                followPath(Trajectories.BreadMetal.Source_MN5, follower),
                followPath(Trajectories.SourceSideFar.MN5_Center, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFar.Center_MN4, follower),
                followPath(Trajectories.SourceSideFar.MN4_Center, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFar.Center_MN3, follower),
                stopDrivetrain(15));
    }
    // --- --- ---

    // --- Source Side Far Dynamic ---
    public Command getSourceSideFarDynamic() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideFarDynamic(follower),
                getSuperstructureSourceSideFarDynamic(follower));
    }

    private Command getSuperstructureSourceSideFarDynamic(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideFarDynamic(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                sequence(
                                followPath(Trajectories.SourceSideFar.sourceFender_MN5, follower),
                                followPath(Trajectories.SourceSideFar.MN5_MN2, follower))
                        .until(superstructure.hasNote),
                followPath(Trajectories.SourceSideFar.Dynamic_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                sequence(
                                followPath(Trajectories.SourceSideFar.SourceTruss_MN4, follower),
                                followPath(Trajectories.SourceSideFar.MN4_MN2, follower))
                        .until(superstructure.hasNote),
                followPath(Trajectories.SourceSideFar.Dynamic_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Source Side Far Flip ---
    public Command getSourceSideFarFlip() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideFarFlip(follower),
                getSuperstructureSourceSideFarFlip(follower));
    }

    private Command getSuperstructureSourceSideFarFlip(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideFarFlip(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideFarFlip.sourceFender_MN4B, follower),
                followPath(Trajectories.SourceSideFarFlip.MN4B_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFarFlip.SourceTruss_MN5B, follower),
                followPath(Trajectories.SourceSideFarFlip.MN5B_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFarFlip.SourceTruss_MN3, follower),
                followPath(Trajectories.SourceSideFarFlip.MN3_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot + 15));
    }
    // --- --- ---

    // --- Source Side Far Flip MN2&3 ---
    public Command getSourceSideFarFlipMN23() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideFarFlipMN23(follower),
                getSuperstructureSourceSideFarFlipMN23(follower));
    }

    private Command getSuperstructureSourceSideFarFlipMN23(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideFarFlipMN23(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideFarFlip.sourceFender_MN4B, follower),
                followPath(Trajectories.SourceSideFarFlip.MN4B_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFarFlip.SourceTruss_MN3, follower),
                followPath(Trajectories.SourceSideFarFlip.MN3_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFarFlip.MN3_SourceTruss, follower),
                stopDrivetrain(15));
    }
    // --- --- ---

    // --- Source Side Far Flip ---
    public Command getSourceSideFarFlipRush() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideFarFlipRush(follower),
                getSuperstructureSourceSideFarFlipRush(follower));
    }

    private Command getSuperstructureSourceSideFarFlipRush(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideFarFlipRush(CustomFollower follower) {
        return sequence(
                parallel(
                        //                        sequence(waitSeconds(rushShotDelay),
                        // superstructure.spitBack()),
                        followPath(Trajectories.SourceSideFarFlip.sourceFender_MN4B, follower)),
                followPath(Trajectories.SourceSideFarFlip.MN4B_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFarFlip.SourceTruss_MN5B, follower),
                followPath(Trajectories.SourceSideFarFlip.MN5B_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideFarFlip.SourceTruss_MN3, follower),
                followPath(Trajectories.SourceSideFarFlip.MN3_SourceTruss, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot + 15));
    }
    // --- --- ---

    // --- Source Side Garbage Collection ---
    public Command getSourceSideGarbageCollection() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getSuperstructureSourceSideGarbageCollection(follower),
                getDrivetrainSourceSideGarbageCollection(follower));
    }

    private Command getSuperstructureSourceSideGarbageCollection(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideGarbageCollection(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideClose.SourceFender_CN3, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.CN3_CN2, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.CN2_CN1, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                Commands.runOnce(() -> setIntakeZoneThreshold(0)),
                followPath(Trajectories.SourceSideClose.CN1_Garbage, follower),
                aimNote(5),
                followPath(Trajectories.SourceSideClose.Garbage_Score, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Source Side Two Note Garbage Collection ---
    public Command getSourceSideTwoNoteGarbageCollection() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainSourceSideTwoNoteGarbageCollection(follower),
                getSuperstructureSourceSideTwoNoteGarbageCollection(follower));
    }

    private Command getSuperstructureSourceSideTwoNoteGarbageCollection(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }

    private Command getDrivetrainSourceSideTwoNoteGarbageCollection(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideClose.SourceFender_CN3, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.CN3_CN2, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.CN2_CN1, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                Commands.runOnce(() -> setIntakeZoneThreshold(0)),
                followPath(Trajectories.SourceSideClose.CN1_Garbage, follower),
                aimNote(5),
                followPath(Trajectories.SourceSideClose.Garbage_Score, follower),
                aimDrivetrain(1),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.SourceSideClose.GarbageScore_GarbageTwo, follower),
                stopDrivetrain(shotDelay + 15));
    }
    // --- --- ---

    // --- Bread Metal ---
    public Command getBreadMetal() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(
                getDrivetrainBreadMetal(follower), getSuperstructureSourceSideFar(follower));
    }

    private Command getDrivetrainBreadMetal(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.BreadMetal.Source_MN5, follower),
                followPath(Trajectories.SourceSideFar.MN5_Center, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideFar.Center_MN4, follower),
                followPath(Trajectories.SourceSideFar.MN4_Center, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot),
                followPath(Trajectories.SourceSideFar.Center_MN3, follower),
                stopDrivetrain(15));
    }

    Command getSuperstructureSourceSide4(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }
    // --- --- ---

    // --- Bear Trap ---
    public Command getBearTrap() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return deadline(getDrivetrainBearTrap(follower), getSuperstructureBearTrap(follower));
    }

    private Command getDrivetrainBearTrap(CustomFollower follower) {
        return sequence(
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.BearTrap.beartrap1, follower),
                followPath(Trajectories.BearTrap.beartrap2, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(shotDelay),
                followPath(Trajectories.BearTrap.beartrap3, follower),
                followPath(Trajectories.BearTrap.beartrap4, follower),
                aimDrivetrain(0.5),
                stopDrivetrain(initialShot + 15));
    }

    Command getSuperstructureBearTrap(CustomFollower follower) {
        return this.getSuperstructureOneCycle(follower).repeatedly();
    }
    // --- --- ---

    public Command getAmpSideTroll() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return getDrivetrainAmpSideTroll(follower);
    }

    private Command getDrivetrainAmpSideTroll(CustomFollower follower) {
        return sequence(followPath(Trajectories.troll.troll1, follower), waitSeconds(15));
    }

    public Command getSourceSideTroll() {
        final CustomFollower follower =
                new CustomFollower(
                        drivetrain::setRequest,
                        noteTracking,
                        () -> Units.degreesToRadians(autoAim.findSpeakerAimVelocity()));
        return getDrivetrainAmpSideTroll(follower);
    }

    private Command getDrivetrainSourceSideTroll(CustomFollower follower) {
        return sequence(followPath(Trajectories.troll.troll2, follower), waitSeconds(15));
    }

    Command getSuperstructureFirstCycleRush(CustomFollower follower) {
        return sequence(
                waitSeconds(rushShotDelay),
                superstructure.spitBack(),
                waitUntil(dtInIntakeZone.or(noteTracking::atNote)),
                follower.aimNote(),
                superstructure.intake().until(superstructure.feederSensor),
                follower.followPath(),
                superstructure.stopAll().withTimeout(0.02));
    }

    Command getSuperstructureOneCycle(CustomFollower follower) {
        return sequence(
                follower.aimSpeaker(),
                superstructure
                        .scoreSpeakerWaitForSpinup(
                                shootingControl::getShooterAngle,
                                shootingControl::getSpeakerVelocity,
                                dtStopped.and(dtInShootingZone))
                        .alongWith(
                                sequence(
                                        waitUntil(dtInShootingZone),
                                        startEnd(
                                                        drivetrain::enableResetToVision,
                                                        drivetrain::disableResetToVision)
                                                .until(dtStopped)
                                                .withTimeout(2))),
                follower.followPath(),
                //                waitUntil(dtInIntakeZone.or(noteTracking::atNote)),
                //                follower.aimNote(),
                waitUntil(dtInIntakeZone),
                superstructure.intake().until(superstructure.feederSensor),
                follower.followPath(),
                superstructure.stopAll().withTimeout(0.02));
    }

    public Command followPath(ChoreoTrajectory trajectory) {
        final ApplyChassisSpeeds autoFollower =
                new ApplyChassisSpeeds().withDriveRequestType(DriveRequestType.Velocity);

        return followPath(
                trajectory,
                robotRelative -> drivetrain.setRequest(autoFollower.withSpeeds(robotRelative)));
    }

    public Command followPath(ChoreoTrajectory trajectory, Consumer<ChassisSpeeds> applySpeeds) {
        return Choreo.choreoSwerveCommand(
                trajectory,
                drivetrain::getOdoPose,
                Choreo.choreoSwerveController(
                        new PIDController(5, 0, 0),
                        new PIDController(5, 0, 0),
                        new PIDController(5, 0, 0)),
                applySpeeds,
                this::shouldFlip,
                drivetrain);
    }

    public Command aimDrivetrain(double timeout) {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();
        return Commands.run(
                        () ->
                                drivetrain.setControl(
                                        request.withRotationalRate(
                                                Units.degreesToRadians(
                                                        autoAim.findSpeakerAimVelocity()))),
                        drivetrain)
                .until(autoAim::isAimed)
                .finallyDo(() -> drivetrain.setControl(request.withRotationalRate(0)))
                .withTimeout(timeout);
    }

    public Command aimNote(double timeout) {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
        return Commands.run(
                        () ->
                                drivetrain.setRequest(
                                        request.withVelocityX(noteTracking.findXVelocity())
                                                .withVelocityY(noteTracking.findYVelocity())
                                                .withRotationalRate(
                                                        noteTracking.findRotationVelocity())),
                        drivetrain)
                .finallyDo(
                        () ->
                                drivetrain.setControl(
                                        request.withVelocityX(0)
                                                .withVelocityY(0)
                                                .withRotationalRate(0)))
                .withTimeout(timeout)
                .until(superstructure.hasNote)
                .onlyIf(noteTracking::atNote);
    }

    public Command drivetrainHeading(DoubleSupplier heading) {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();
        final ProfiledPIDController kAmpAimYawControllers =
                new ProfiledPIDController(12, 0, 1.2, new TrapezoidProfile.Constraints(458, 916));
        kAmpAimYawControllers.enableContinuousInput(-180, 180);
        kAmpAimYawControllers.setTolerance(2, 3);
        return Commands.run(
                        () ->
                                drivetrain.setControl(
                                        request.withRotationalRate(
                                                Units.degreesToRadians(
                                                        kAmpAimYawControllers.calculate(
                                                                drivetrain.getHeading(),
                                                                heading.getAsDouble())))),
                        drivetrain)
                .until(kAmpAimYawControllers::atSetpoint)
                .finallyDo(() -> drivetrain.setControl(request.withRotationalRate(0)));
    }

    public Command stopDrivetrain(double timeout) {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();
        return Commands.run(
                        () ->
                                drivetrain.setControl(
                                        request.withVelocityX(0)
                                                .withVelocityY(0)
                                                .withRotationalRate(0)),
                        drivetrain)
                .withTimeout(timeout);
    }

    public void onAllianceFound(Alliance alliance) {
        this.alliance = alliance;
    }

    public boolean shouldFlip() {
        return alliance == Alliance.Red;
    }

    @Override
    public Iterator<AutoMode> iterator() {
        return modes.iterator();
    }
}

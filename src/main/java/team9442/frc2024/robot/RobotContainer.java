// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team9442.frc2024.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;
import monologue.Monologue;
import team9442.frc2024.alignment.AutoAim;
import team9442.frc2024.alignment.NoteTracking;
import team9442.frc2024.autonomous.AutoChooser;
import team9442.frc2024.autonomous.AutoCommands;
import team9442.frc2024.commands.DrivetrainCommands;
import team9442.frc2024.commands.LeftRightShooter;
import team9442.frc2024.commands.TestCommands;
import team9442.frc2024.constants.*;
import team9442.frc2024.subsystems.*;
import team9442.frc2024.util.RumbleControl;
import team9442.frc2024.util.SensorSupplier;
import team9442.frc2024.vision.LookaheadCalculator;
import team9442.frc2024.vision.NoteDetectionPV;
import team9442.frc2024.vision.Targeting;
import team9442.frc2024.vision.Vision;
import team9442.frc2024.vision.Vision.CamToEstimator;
import team9442.lib.AllianceChecker;
import team9442.lib.Joysticks;
import team9442.lib.RepeatCommandWithEnd;
import team9442.lib.VirtualSubsystem;

public class RobotContainer extends VirtualSubsystem implements Logged {

    public RobotContainer() {
        CommandScheduler.getInstance()
                .registerSubsystem(
                        drivetrain,
                        intake,
                        feeder,
                        indexer,
                        ampPivot,
                        ampRollers,
                        shooterPivot,
                        starboardSideRollers,
                        portSideRollers,
                        telescopeLeft,
                        telescopeRight);
        configureBindings();
        configDefaultCommands();
        shooterPivot.resetEncoder();
        ampPivot.resetEncoder();
        drivetrain.setRobotPose(new Pose2d());
        drivetrain.setToBrake();
        Monologue.setupMonologue(this, "Robot", false, false);
        autoAim.enable();
        setLEDTriggers();
        shootingControl.setAutoScoring();
        allianceChecker.registerObservers(targeting, autoChooser, autoCommands, drivetrain);
    }

    public void periodic() {
        this.log("autoaim enabled", autoAim.isEnabled());
        this.log("is aimed", autoAim.isAimed());
        this.log("drivetrain pose2d", drivetrain.getOdoPose());
        field.setRobotPose(drivetrain.getOdoPose());
        field.getObject("target").setPose(targeting.getFieldToSpeakerWithOffset());
        this.log("drivetrain vx", drivetrain.getRobotVx());
        this.log("should flip", autoCommands.shouldFlip());
        this.log("target", targeting.getFieldToSpeakerWithOffset());
        this.log("target apriltag", targeting.getFieldToCenterTag());
        this.log(
                "distance to target",
                lackAiming.getEffectiveRangeToTarget(),
                LogLevel.OVERRIDE_FILE_ONLY);
        this.log("dt in teleop score zone", dtInScoreTeleopScoreZone.getAsBoolean());
        this.log("aim error", DrivetrainConstants.kRotControllerDegrees.getPositionError());
        this.log("amp aim error", DrivetrainConstants.kAmpAimYawControllers.getPositionError());
        this.log(
                "pivot at shooting setpoint",
                pivotAtShootingControlSetpoint.getAsBoolean(),
                LogLevel.OVERRIDE_FILE_ONLY);
        this.log("pivot error", shootingControl.getShooterAngle() - shooterPivot.getPosition());
        this.log("intake sensor", IntakeConstants.kIntakeSensorTrigger.getAsBoolean());
        this.log("staging sensor", FeederConstants.kStagingSensorTrigger.getAsBoolean());
        log("right climber", TelescopeConstants.kRight.getEncoder().getPosition());
        log("left climber", TelescopeConstants.kLeft.getEncoder().getPosition());
        log("climbing mode", superstructure.climbingMode.getAsBoolean());
        log("amp rollers current", ampRollers.getMasterCurrent());
    }

    private void configDefaultCommands() {
        drivetrain.setDefaultCommand(
                drivetrainCommands.drive(
                        mainController::getLeftStickY,
                        () -> -mainController.getLeftStickX(),
                        () -> -mainController.getRightStickX(),
                        () -> (noteDetectionPV.getTy() + 25) / 50,
                        autoAim::findSpeakerAimVelocity,
                        shooting,
                        autoAim::findAmpAimVelocity,
                        mainController.rightBumper,
                        noteTracking::findRotationVelocity,
                        intaking.and(noteTracking::isEnabled)
                                .and(
                                        new Trigger(noteTracking::atNote)
                                                .debounce(0.5, Debouncer.DebounceType.kFalling)),
                        noteDetectionPV,
                        () -> true,
                        () -> false));
        starboardSideRollers.setDefaultCommand(
                superstructure.spinIfNote(
                        starboardSideRollers,
                        shootingControl::getSpeakerVelocity,
                        operatorSpinupButtons,
                        dtInScoreTeleopScoreZone.and(intaking.negate()),
                        0.7));
        portSideRollers.setDefaultCommand(
                superstructure.spinIfNote(
                        portSideRollers,
                        shootingControl::getSpeakerVelocity,
                        operatorSpinupButtons,
                        dtInScoreTeleopScoreZone.and(intaking.negate()),
                        1));
        shooterPivot.setDefaultCommand(
                superstructure.pivotAngleIfNoteAndPreAim(
                        shootingControl::getShooterAngle,
                        dtInScoreTeleopScoreZone.and(dtInScoreTeleopScoreZone)));
        feeder.setDefaultCommand(superstructure.feederRollbackDefault());
        indexer.setDefaultCommand(indexer.openloop(0));
        intake.setDefaultCommand(intake.openloop(0));
        ampPivot.setDefaultCommand(ampPivot.holdAtCall());
        telescopeLeft.setDefaultCommand(telescopeLeft.openloop(0));
        telescopeRight.setDefaultCommand(telescopeRight.openloop(0));
        // ampRollers.setDefaultCommand(ampRollers.openloop(() -> coController.getLeftStickY() *
        // 0.5));
    }

    private void configureBindings() {
        intaking.and(superstructure.feederSensor.negate()).whileTrue(superstructure.intake());
        intaking.whileTrue(
                sequence(
                                waitSeconds(0.5),
                                waitUntil(feederSensor),
                                new RepeatCommandWithEnd(mainControllerRumble.rumble(0.7, 0.1), 3))
                        .finallyDo(mainControllerRumble.stopRumble));

        mainController.leftBumper.whileTrue(superstructure.spit());
        shooting.whileTrue(
                        superstructure.scoreSpeakerWaitForSpinup(
                                shootingControl::getShooterAngle,
                                shootingControl::getSpeakerVelocity,
                                new Trigger(autoAim::isAimed).debounce(0.1)))
                .whileTrue(
                        startEnd(drivetrain::enableResetToVision, drivetrain::disableResetToVision)
                                .withTimeout(0.2));
        mainController.buttonB.onTrue(
                runOnce(
                        () ->
                                drivetrain.setRobotPose(
                                        new Pose2d(
                                                drivetrain.getOdoPose().getTranslation(),
                                                new Rotation2d()))));

        mainController.rightMidButton.onTrue(runOnce(autoAim::enable));
        mainController.leftMidButton.onFalse(runOnce(autoAim::disable));

        mainController.rightBumper.whileTrue(
                parallel(
                        superstructure.ampHandoffToPosition(),
                        startEnd(() -> autoAim.lockHeading(90), autoAim::stopHeadingLock)));
        mainController.rightBumper.onFalse(superstructure.scoreAmpAndStow());
        mainController.buttonX.onTrue(ampPivot.angle(250));

        mainController.dPadDown.onTrue(shootingControl.decreaseAngleAdjustment());
        mainController.dPadUp.onTrue(shootingControl.increaseAngleAdjustment());
        mainController.dPadLeft.onTrue(shootingControl.decreaseVelocityAdjustment());
        mainController.dPadRight.onTrue(shootingControl.increaseVelocityAdjustment());

        climbingUp.whileTrue(superstructure.climbersUp());
        climbingDown.whileTrue(superstructure.climbersDown());

        coController.leftBumper.whileTrue(superstructure.spitBack());

        coController
                .buttonA
                .onTrue(shootingControl.setFenderScoring())
                .onTrue(runOnce(autoAim::disable));
        coController
                .buttonB
                .onTrue(shootingControl.setAutoScoring())
                .onTrue(runOnce(autoAim::enable));
        coController.buttonX.onTrue(shootingControl.setProtectedScoring());

        coController.leftStickHeldUp.onTrue(ampRollers.openloop(0.1).withTimeout(0.3));
        coController.leftStickHeldDown.onTrue(ampRollers.openloop(-0.5).withTimeout(0.5));

        coController.leftMidButton.onTrue(noteTracking.disableC());
        coController.rightMidButton.onTrue(noteTracking.enableC());

        coController.rightBumper.whileTrue(shooterPivot.currentBasedHome());
        coController.buttonY.whileTrue(
                startEnd(drivetrain::enableResetToVision, drivetrain::disableResetToVision));
        dtTeleStopped.whileTrue(
                startEnd(drivetrain::enableResetToVision, drivetrain::disableResetToVision)
                        .withTimeout(0.04));

        coController.dPadUp.onTrue(superstructure.setClimbMode(true));
        coController.dPadDown.onTrue(superstructure.setClimbMode(false));
    }

    private void setLEDTriggers() {
        noNoteAndNoTracking.onTrue(leds.solid(LEDConstants.kRedBuffer));
        noNoteAndTracking.onTrue(leds.flash(LEDConstants.kRedBuffer));
        noNoteButAmp.onTrue(leds.flash(LEDConstants.kPurpleBuffer));
        noteAndFar.onTrue(leds.flash(LEDConstants.kYellowBuffer));
        noteAndCloseNotAimed.onTrue(leds.flash(LEDConstants.kWhiteBuffer));
        noteAndCloseAimed.onTrue(leds.flash(LEDConstants.kGreenBuffer));
        noteAndCloseAimedAndPivotShooters.onTrue(leds.solid(LEDConstants.kGreenBuffer));
    }

    public Command getAutonomousCommand() {
        final var selectedMode = autoChooser.getSelected();
        if (selectedMode == null) {
            return none();
        }
        return selectedMode.command;
    }

    public Command getTestCommand() {
        return new TestCommands(superstructure, drivetrainCommands, () -> 1).test();
    }

    private final Joysticks mainController = new Joysticks(0);
    private final RumbleControl mainControllerRumble = new RumbleControl(mainController::rumble);
    private final Joysticks coController = new Joysticks(1);
    private final Joysticks testController = new Joysticks(2);

    final Drivetrain drivetrain =
            new Drivetrain(
                    TunerConstants.kDrivetrainConstants,
                    TunerConstants.kSpeedAt12VoltsMps,
                    TunerConstants.kMaxPossibleRadPerSec,
                    TunerConstants.kFrontLeft,
                    TunerConstants.kFrontRight,
                    TunerConstants.kBackLeft,
                    TunerConstants.kBackRight);

    private final SparkRollers intake =
            new SparkRollers(
                    IntakeConstants.kIntake, IntakeConstants.kConstants, GlobalConstants.kDt);
    private final SparkRollers indexer =
            new SparkRollers(
                    IntakeConstants.kIndexer, IntakeConstants.kConstants, GlobalConstants.kDt);
    private final SparkPivot ampPivot =
            new SparkPivot(
                    AmpPivotConstants.kMaster, AmpPivotConstants.kConstants, GlobalConstants.kDt);
    private final SparkRollers ampRollers =
            new SparkRollers(
                    AmpRollersConstants.kMaster,
                    AmpRollersConstants.kConstants,
                    GlobalConstants.kDt);
    private final Pivot shooterPivot =
            new Pivot(
                    ShooterPivotConstants.kMaster,
                    ShooterPivotConstants.kConstants,
                    GlobalConstants.kDt);
    private final Rollers feeder =
            new Rollers(FeederConstants.kMaster, FeederConstants.kConstants, GlobalConstants.kDt);
    private final Rollers portSideRollers =
            new Rollers(
                    ShooterRollersConstants.kPortSide,
                    ShooterRollersConstants.kConstants,
                    GlobalConstants.kDt);
    private final Rollers starboardSideRollers =
            new Rollers(
                    ShooterRollersConstants.kStarboardSide,
                    ShooterRollersConstants.kConstants,
                    GlobalConstants.kDt);
    private final LeftRightShooter shooterCommands =
            new LeftRightShooter(portSideRollers, starboardSideRollers, 0.7);

    private final Telescope telescopeRight =
            new Telescope(
                    TelescopeConstants.kRight, TelescopeConstants.kConstants, GlobalConstants.kDt);
    private final Telescope telescopeLeft =
            new Telescope(
                    TelescopeConstants.kLeft, TelescopeConstants.kConstants, GlobalConstants.kDt);
    private final ClimberSuperstucture climber =
            new ClimberSuperstucture(telescopeRight, telescopeLeft);

    private final Targeting targeting =
            new Targeting(
                    drivetrain::getOdoPose,
                    VisionConstants.kTagLayout.getTagPose(7).get().toPose2d(),
                    VisionConstants.kTagLayout.getTagPose(4).get().toPose2d(),
                    Units.inchesToMeters(6));

    private final LookaheadCalculator lackAiming =
            new LookaheadCalculator(
                    drivetrain::getFieldRelativeChassisSpeeds,
                    drivetrain::getOdoRot,
                    targeting::getLatestParameters,
                    () -> false,
                    0);

    private final LEDs leds = new LEDs(LEDConstants.kLEDs, LEDConstants.kBlackBuffer);

    private final DrivetrainCommands drivetrainCommands = new DrivetrainCommands(drivetrain);
    final Vision vision =
            new Vision(
                    List.of(
                            //                            new CamToEstimator(
                            //                                    VisionConstants.portCam,
                            // VisionConstants.portEstimator),
                            //     new CamToEstimator(
                            //             VisionConstants.starboardCam,
                            //             VisionConstants.starboardEstimator),
                            new CamToEstimator(
                                    VisionConstants.centerCam, VisionConstants.centerEstimator)),
                    drivetrain::updateEstimates);
    final NoteDetectionPV noteDetectionPV = new NoteDetectionPV(VisionConstants.noteCam);
    final NoteTracking noteTracking =
            new NoteTracking(
                    DrivetrainConstants.kNoteYawController,
                    DrivetrainConstants.kNoteXController,
                    DrivetrainConstants.kNoteYController,
                    noteDetectionPV);
    private final AutoAim autoAim =
            new AutoAim(
                    DrivetrainConstants.kAmpAimYawControllers,
                    DrivetrainConstants.kRotControllerDegrees,
                    lackAiming::getLatestAimingParameters,
                    drivetrain::getOdoRot);

    final AllianceChecker allianceChecker = new AllianceChecker();
    final ShootingControl shootingControl =
            new ShootingControl(lackAiming::getEffectiveRangeToTarget);

    private final SensorSupplier feederSensor =
            new SensorSupplier(
                    FeederConstants.kFeederSensorTrigger,
                    feeder::getVelocity,
                    feeder::getMasterCurrent);
    private final Superstructure superstructure =
            new Superstructure(
                    intake,
                    indexer,
                    feeder,
                    shooterPivot,
                    shooterCommands,
                    ampRollers,
                    ampPivot,
                    climber,
                    new Trigger(feederSensor));

    private final AutoCommands autoCommands =
            new AutoCommands(drivetrain, superstructure, noteTracking, autoAim, shootingControl);

    @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
    private final AutoChooser autoChooser = new AutoChooser(autoCommands, drivetrain::setRobotPose);

    @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
    private final Field2d field = new Field2d();

    private final Trigger intaking =
            mainController.leftTrigger.and(superstructure.climbingMode.negate());
    private final Trigger shooting =
            mainController.rightTrigger.and(superstructure.climbingMode.negate());
    private final Trigger climbingDown =
            mainController.leftTrigger.and(superstructure.climbingMode);
    private final Trigger climbingUp = mainController.rightTrigger.and(superstructure.climbingMode);
    private final Trigger dtInScoreTeleopScoreZone =
            new Trigger(() -> lackAiming.getRawRangeToTarget() < 5.22);
    private final Trigger pivotAtShootingControlSetpoint =
            new Trigger(
                    () -> shooterPivot.atPositionSetpoint(shootingControl.getShooterAngle(), 0.5));
    private final Trigger noNoteAndNoTracking =
            superstructure
                    .feederSensor
                    .negate()
                    .and(mainController.rightBumper.negate())
                    .and(new Trigger(noteTracking::atNote).negate());
    private final Trigger noNoteAndTracking =
            superstructure
                    .feederSensor
                    .negate()
                    .and(mainController.rightBumper.negate())
                    .and(noteTracking::atNote);
    private final Trigger noNoteButAmp =
            superstructure.feederSensor.negate().and(mainController.rightBumper);
    private final Trigger noteAndFar =
            superstructure.feederSensor.and(dtInScoreTeleopScoreZone.negate());
    private final Trigger noteAndCloseNotAimed =
            superstructure
                    .feederSensor
                    .and(dtInScoreTeleopScoreZone)
                    .and(new Trigger(autoAim::isAimed).negate());
    private final Trigger noteAndCloseAimed =
            superstructure
                    .feederSensor
                    .and(dtInScoreTeleopScoreZone)
                    .and(mainController.rightBumper.negate())
                    .and(pivotAtShootingControlSetpoint.negate())
                    .and(new Trigger(shooterCommands::bothAtVelocitySetpoint).negate())
                    .and(autoAim::isAimed);
    private final Trigger noteAndCloseAimedAndPivotShooters =
            superstructure
                    .feederSensor
                    .and(dtInScoreTeleopScoreZone)
                    .and(mainController.rightBumper.negate())
                    .and(pivotAtShootingControlSetpoint)
                    .and(shooterCommands::bothAtVelocitySetpoint)
                    .and(autoAim::isAimed);
    private final Trigger operatorSpinupButtons =
            coController.buttonA.or(coController.buttonB).or(coController.buttonX);
    private final Trigger dtTeleStopped =
            new Trigger(() -> drivetrain.getVel() < 0.3).debounce(0.5);
}

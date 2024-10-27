package team9442.frc2024.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.LogLevel;
import monologue.Logged;
import team9442.frc2024.commands.LeftRightShooter;
import team9442.frc2024.constants.AmpPivotConstants;
import team9442.frc2024.constants.FeederConstants;
import team9442.frc2024.constants.IntakeConstants;
import team9442.frc2024.constants.ShooterPivotConstants;
import team9442.frc2024.constants.TelescopeConstants;
import team9442.frc2024.util.SpeakerScoreType;
import team9442.lib.VirtualSubsystem;

public class Superstructure extends VirtualSubsystem implements Logged {

    public void periodic() {
        this.log("feeder sensor", feederSensor.getAsBoolean(), LogLevel.OVERRIDE_FILE_ONLY);
        this.log(
                "shooters at setpoint",
                shooter.bothAtVelocitySetpoint(),
                LogLevel.OVERRIDE_FILE_ONLY);
    }

    public Command intake() {
        return parallel(
                shooterPivot.angle(ShooterPivotConstants.kGroundIntakeAngle),
                shooter.openloopZero(),
                intake.openloop(-1),
                indexer.openloop(0.9),
                feeder.openloop(0.3).until(FeederConstants.kFeederSensorTrigger));
    }

    public Command stopAll() {
        return parallel(feeder.openloop(0), intake.openloop(0), indexer.openloop(0))
                .withTimeout(0.01);
    }

    public Command stationIntake() {
        return parallel(
                ampPivot.angle(AmpPivotConstants.kAmpLoadingStationAngle),
                ampRollers.openloop(0.3));
    }

    public Command feederRollbackDefault() {
        return sequence(
                sequence(
                        // feeder.openloop(-0.3).until(feederSensor.negate()).withTimeout(0.2),
                        feeder.openloop(0.2)
                                .onlyIf(FeederConstants.kFeederSensorTrigger)
                                .until(FeederConstants.kStagingSensorTrigger),
                        feeder.openloop(0)));
    }

    public Command feederRollbackDefaultWithEnd() {
        return sequence(
                sequence(
                        // feeder.openloop(-0.3).until(feederSensor.negate()).withTimeout(0.2),
                        feeder.openloop(0.2).until(feederSensor).withTimeout(0.0)),
                feeder.openloop(0).withTimeout(0.02));
    }

    public Command spit() {
        return sequence(
                shooterPivot
                        .angle(15)
                        .until(() -> shooterPivot.atPositionSetpoint(15, 6))
                        .withTimeout(1),
                parallel(intake.openloop(1), indexer.openloop(-1), feeder.openloop(-0.5)));
    }

    public Command spitBack() {
        return sequence(
                shooterPivot
                        .angle(15)
                        .until(() -> shooterPivot.atPositionSetpoint(15, 6))
                        .withTimeout(1),
                parallel(
                                shooter.openloopEquals(0.5),
                                intake.openloop(1),
                                indexer.openloop(1),
                                feeder.openloop(0.5))
                        .withTimeout(1));
    }

    public Command ampHandoff() {
        return race(
                shooterPivot.angle(ShooterPivotConstants.kHandoffAngle),
                ampPivot.angle(AmpPivotConstants.kHandoffAngle),
                shooter.openloopEquals(0.2),
                sequence(
                        waitUntil(
                                () ->
                                        shooterPivot.atPositionSetpoint(
                                                        ShooterPivotConstants.kHandoffAngle, 4)
                                                && ampPivot.atPositionSetpoint(
                                                        AmpPivotConstants.kHandoffAngle, 6)
                                                && Math.abs(shooter.getVelocityAverage()) < 10
                                                && Math.abs(shooter.getVelocityAverage()) > 2),
                        race(
                                feeder.openloop(0.3),
                                ampRollers.openloop(-0.1),
                                sequence(
                                        waitUntil(feederSensor),
                                        waitUntil(feederSensor.negate()),
                                        waitSeconds(0.15)))));
    }

    public Command ampIntake() {
        return parallel(
                race(
                        // Get shooter ready
                        shooterPivot.angle(ShooterPivotConstants.kHandoffAngle),
                        shooter.openloopEquals(-0.2),
                        sequence(waitUntil(feederSensor), waitSeconds(0.5))),
                sequence(
                        race(
                                ampPivot.angle(AmpPivotConstants.kAmpIntakeAngle),
                                ampRollers.openloop(-0.3),
                                sequence(
                                        //
                                        // waitUntil(ampRollers.getMasterCurrent()),
                                        waitSeconds(6))),
                        // Wait until shooter ready.
                        waitUntil(
                                () ->
                                        shooterPivot.atPositionSetpoint(
                                                        ShooterPivotConstants.kHandoffAngle, 4)
                                                && Math.abs(shooter.getVelocityAverage()) < -2
                                                && Math.abs(shooter.getVelocityAverage()) > -10),
                        // Move amp pivot.
                        ampPivot.angle(AmpPivotConstants.kHandoffAngle)
                                .until(
                                        () ->
                                                ampPivot.atPositionSetpoint(
                                                        AmpPivotConstants.kHandoffAngle, 6)),
                        // Put note in shooter.
                        race(
                                ampRollers.openloop(0.3),
                                feeder.openloop(-0.3),
                                sequence(waitUntil(feederSensor), waitSeconds(0.5))),
                        // Stage note.
                        feeder.openloop(0.3).until(feederSensor)));
    }

    public Command ampHandoffToPosition() {
        return sequence(
                ampHandoff(),
                ampPivot.angle(AmpPivotConstants.kAmpScoreAngle)
                        .until(
                                () ->
                                        ampPivot.atPositionSetpoint(
                                                AmpPivotConstants.kAmpScoreAngle, 5)));
    }

    public Command spinIfNote(
            Rollers shooterRollers,
            DoubleSupplier surfaceVelocity,
            Trigger allowSpinup,
            Trigger forceSpinup,
            double ratio) {

        final Trigger allowSpinupAndNoteIfFeederIsStaged =
                forceSpinup.and(feederSensor.debounce(1)).or(allowSpinup);
        return sequence(
                        shooterRollers.openloop(0).until(allowSpinupAndNoteIfFeederIsStaged),
                        shooterRollers
                                .velocity(
                                        () ->
                                                Math.min(
                                                        ratio * surfaceVelocity.getAsDouble(),
                                                        ratio
                                                                * SpeakerScoreType.kFender
                                                                        .getVelocity()))
                                .until(allowSpinupAndNoteIfFeederIsStaged.negate()))
                .repeatedly();
    }

    public Command scoreSpeakerWaitForSpinup(
            DoubleSupplier pivotAngle, DoubleSupplier surfaceVelocity, BooleanSupplier allowShoot) {
        final var shooterAtVelDebounce = new Trigger(shooter::bothAtVelocitySetpoint).debounce(0.1);
        return scoreSpeaker(
                pivotAngle,
                surfaceVelocity,
                () ->
                        allowShoot.getAsBoolean()
                                && shooterPivot.atPositionSetpoint(
                                        pivotAngle.getAsDouble(),
                                        pivotAngle.getAsDouble() > 50 ? 1 : 0.5)
                                && shooterAtVelDebounce.getAsBoolean());
    }

    public Command scoreSpeaker(
            DoubleSupplier pivotAngle, DoubleSupplier surfaceVelocity, BooleanSupplier allowShoot) {
        return parallel(
                        shooterPivot.angle(pivotAngle),
                        shooter.velocity(surfaceVelocity),
                        sequence(waitUntil(allowShoot), feeder.openloop(0.3)),
                        sequence(waitUntil(allowShoot), intake.openloop(-1)),
                        sequence(waitUntil(allowShoot), indexer.openloop(1)))
                .raceWith(
                        sequence(
                                waitUntil(allowShoot),
                                waitUntil(feederSensor.negate().debounce(0.4)),
                                waitSeconds(0.7),
                                waitSeconds(0.7)
                                        .onlyIf(
                                                IntakeConstants.kIntakeSensorTrigger.or(
                                                        feederSensor))));
    }

    public Command scoreAmpAndStow() {
        return sequence(
                this.scoreAmp(),
                parallel(
                                ampPivot.angle(AmpPivotConstants.kAmpScoreAngle - 10),
                                ampRollers.openloop(-0.3))
                        .withTimeout(1),
                waitSeconds(3),
                ampPivot.angle(250));
    }

    public Command scoreAmp() {
        return sequence(
                ampPivot.angleWithEnd(AmpPivotConstants.kAmpScoreAngle),
                ampRollers.openloop(-0.3).withTimeout(1));
    }

    public Command pivotAngleIfNoteAndPreAim(DoubleSupplier aimedPivotAngle, Trigger preAim) {
        final Trigger preAimAndNote = new Trigger(preAim).and(feederSensor);
        return sequence(
                        sequence(
                                        shooterPivot
                                                .angle(ShooterPivotConstants.kConstants.minPosition)
                                                .until(
                                                        () ->
                                                                shooterPivot.atPositionSetpoint(
                                                                        ShooterPivotConstants
                                                                                .kConstants
                                                                                .minPosition,
                                                                        5)),
                                        shooterPivot.openloop(0))
                                .repeatedly()
                                .until(preAimAndNote),
                        shooterPivot.angle(aimedPivotAngle).until(preAimAndNote.negate()))
                .repeatedly();
    }

    public Command climbersUp() {
        return climber.openLoopBothWithEnd(() -> TelescopeConstants.climberSpeed);
    }

    public Command climbersDown() {
        return climber.openLoopBothWithEnd(() -> -TelescopeConstants.climberSpeed);
    }

    public Command setClimbMode(boolean climbMode) {
        return Commands.runOnce(() -> this.isClimbing = climbMode);
    }

    public Superstructure(
            SparkRollers intake,
            SparkRollers indexer,
            Rollers feeder,
            Pivot shooterPivot,
            LeftRightShooter shooter,
            SparkRollers ampRollers,
            SparkPivot ampPivot,
            ClimberSuperstucture climber,
            Trigger feederSensor) {
        this.intake = intake;
        this.indexer = indexer;
        this.feeder = feeder;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.ampRollers = ampRollers;
        this.ampPivot = ampPivot;
        this.climber = climber;
        this.feederSensor = feederSensor;
        this.hasNote = feederSensor.or(IntakeConstants.kIntakeSensorTrigger);
        this.climbingMode = new Trigger(() -> this.isClimbing);
    }

    public final SparkRollers intake;
    public final SparkRollers indexer;

    public final Rollers feeder;
    public final Pivot shooterPivot;
    public final LeftRightShooter shooter;

    public final SparkRollers ampRollers;
    public final SparkPivot ampPivot;

    private final ClimberSuperstucture climber;

    public final Trigger feederSensor;

    private boolean isClimbing = false;
    public final Trigger hasNote;
    public final Trigger climbingMode;
}

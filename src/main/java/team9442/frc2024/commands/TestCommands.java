package team9442.frc2024.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team9442.frc2024.constants.AmpPivotConstants;
import team9442.frc2024.constants.ShooterPivotConstants;
import team9442.frc2024.subsystems.Superstructure;
import team9442.lib.VirtualSubsystem;

public class TestCommands extends VirtualSubsystem {
    enum TestState {
        TURN_DRIVETRAIN,
        SHOOTER_PIVOT,
        SHOOTER_ROLLERS,
        FEEDER,
        INDEXER,
        INTAKE,
        AMP_PIVOT,
        AMP_ROLLERS,
        CLIMBER
    }

    Superstructure superstructure;
    DrivetrainCommands dtCommands;
    DoubleSupplier timeout;
    TestState state = TestState.values()[0];

    public TestCommands(
            Superstructure superstructure, DrivetrainCommands dtCommands, DoubleSupplier timeout) {
        this.superstructure = superstructure;
        this.dtCommands = dtCommands;
        this.timeout = timeout;
    }

    @Override
    public void periodic() {
        superstructure.log("test state", this.state.toString());
    }

    public Command test() {
        return sequence(
                runOnce(() -> state = TestState.TURN_DRIVETRAIN),
                testDrivetrainRotation(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.SHOOTER_PIVOT),
                testShooterPivot(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.SHOOTER_ROLLERS),
                testShooterRollers(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.FEEDER),
                testFeeder(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.INDEXER),
                testIndexer(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.INTAKE),
                testIntake(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.AMP_PIVOT),
                testAmpPivot(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.AMP_ROLLERS),
                testAmpRollers(),
                waitSeconds(timeout.getAsDouble()),
                runOnce(() -> state = TestState.CLIMBER),
                testClimbers());
    }

    public Command testDrivetrainRotation() {
        return sequence(
                dtCommands.rotate(() -> 0.25).withTimeout(5),
                dtCommands.rotate(() -> 0).withTimeout(0.02));
    }

    public Command testShooterPivot() {
        return sequence(
                superstructure
                        .shooterPivot
                        .angle(ShooterPivotConstants.kGroundIntakeAngle)
                        .withTimeout(1),
                superstructure
                        .shooterPivot
                        .angle(ShooterPivotConstants.kHandoffAngle)
                        .withTimeout(1),
                superstructure
                        .shooterPivot
                        .angle(ShooterPivotConstants.kSpeakerFarTrussScoreAngle)
                        .withTimeout(1));
    }

    public Command testShooterRollers() {
        return sequence(
                superstructure.shooter.openloopEquals(1).withTimeout(2),
                superstructure.shooter.openloopZero().withTimeout(0.02));
    }

    public Command testFeeder() {
        return sequence(superstructure.feeder.openloop(1).withTimeout(2), superstructure.stopAll());
    }

    public Command testIndexer() {
        return sequence(
                superstructure.indexer.openloop(1).withTimeout(2), superstructure.stopAll());
    }

    public Command testIntake() {
        return sequence(superstructure.intake.openloop(1).withTimeout(2), superstructure.stopAll());
    }

    public Command testAmpPivot() {
        return sequence(
                superstructure.ampPivot.angle(AmpPivotConstants.kAmpScoreAngle).withTimeout(1),
                superstructure.ampPivot.angle(AmpPivotConstants.kHandoffAngle).withTimeout(1),
                superstructure.ampPivot.angle(AmpPivotConstants.kAmpScoreAngle).withTimeout(1),
                superstructure.ampPivot.angle(250).withTimeout(1.5));
    }

    public Command testAmpRollers() {
        return sequence(
                superstructure.ampRollers.openloop(1).withTimeout(2), superstructure.stopAll());
    }

    public Command testClimbers() {
        return sequence(
                superstructure.climbersUp().withTimeout(5),
                superstructure.climbersDown().withTimeout(5));
    }
}

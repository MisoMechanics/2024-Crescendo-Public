package team9442.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public final class CustomSwerveRequests {
    private static final MotionMagicVoltage motionMagicControl =
            new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);

    public static class CharacterizeTurn implements SwerveRequest {
        private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
        public double targetTorque = 0.0;

        @Override
        public StatusCode apply(
                SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (final var module : modulesToApply) {
                module.getSteerMotor()
                        .setControl(
                                torqueCurrentFOC.withOutput(targetTorque).withMaxAbsDutyCycle(0.7));
                module.getDriveMotor().setControl(motionMagicControl);
            }

            return StatusCode.OK;
        }

        public CharacterizeTurn withTargetTorque(double targetTorque) {
            this.targetTorque = targetTorque;
            return this;
        }
    }

    public static class CharacterizeDrive implements SwerveRequest {
        private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
        public double targetTorque = 0.0;

        @Override
        public StatusCode apply(
                SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (final var module : modulesToApply) {
                module.getDriveMotor()
                        .setControl(
                                torqueCurrentFOC.withOutput(targetTorque).withMaxAbsDutyCycle(0.7));
                module.getSteerMotor().setControl(motionMagicControl);
            }

            return StatusCode.OK;
        }

        public CharacterizeDrive withTargetTorque(double targetTorque) {
            this.targetTorque = targetTorque;
            return this;
        }
    }
}

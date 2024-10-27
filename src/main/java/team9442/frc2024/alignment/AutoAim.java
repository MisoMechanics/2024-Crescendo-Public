package team9442.frc2024.alignment;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import team9442.frc2024.vision.LookaheadCalculator.LookaheadAimingParameters;

public class AutoAim {

    private final ProfiledPIDController ampController;
    private final ProfiledPIDController profiledThetaController;
    private final Supplier<Rotation2d> robotYaw;
    private final Supplier<LookaheadAimingParameters> getAimingParams;

    private boolean enabled = false;
    private double headingLock = 0;
    private boolean headingLockEnable = false;

    public AutoAim(
            ProfiledPIDController ampController,
            ProfiledPIDController profiledThetaController,
            Supplier<LookaheadAimingParameters> getAimingParams,
            Supplier<Rotation2d> robotYaw) {
        this.ampController = ampController;
        this.profiledThetaController = profiledThetaController;
        this.getAimingParams = getAimingParams;
        this.robotYaw = robotYaw;
    }

    public void enable() {
        this.enabled = true;
    }

    public void disable() {
        this.enabled = false;
    }

    public boolean isEnabled() {
        return this.enabled;
    }

    public void lockHeading(double heading) {
        this.headingLock = heading;
        this.headingLockEnable = true;
    }

    public void stopHeadingLock() {
        this.headingLockEnable = false;
    }

    public double findAmpAimVelocity() {
        return ampController.calculate(robotYaw.get().getDegrees(), 90);
    }

    public double findSpeakerAimVelocity() {
        var rotationVelocity = 0.0;
        final var params = getAimingParams.get();
        if (!enabled || params == null) {
            rotationVelocity = 0;
        } else {
            rotationVelocity =
                    profiledThetaController.calculate(
                                    robotYaw.get().getDegrees(), params.effectiveYawSetpoint())
                            + params.effectiveYawFeedforwardVelocityDegPS();
        }

        return rotationVelocity;
    }

    public boolean isAimedAmp() {
        return ampController.atSetpoint() || !this.enabled;
    }

    public boolean isAimed() {
        return profiledThetaController.atSetpoint() || !this.enabled;
    }
}

package team9442.lib;

import java.util.function.BiFunction;

public class ServoConstants {
    public final double velocityTolerance;
    public final double positionTolerance;

    public final double velocityConversion;
    public final double positionConversion;

    public final double minPosition;
    public final double maxPosition;
    public final double resetPosition;

    public final BiFunction<Double, Double, Double> feedforward;

    public final double nominalVoltage;

    public final double homingCurrentAmps;
    public final double homingSpeed;

    private ServoConstants(
            double velocityTolerance,
            double positionTolerance,
            double velocityConversion,
            double positionConversion,
            double minPosition,
            double maxPosition,
            double resetPosition,
            BiFunction<Double, Double, Double> feedforward,
            double nominalVoltage,
            double homingCurrentAmps,
            double homingSpeed) {
        this.velocityTolerance = velocityTolerance;
        this.positionTolerance = positionTolerance;
        this.velocityConversion = velocityConversion;
        this.positionConversion = positionConversion;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.resetPosition = resetPosition;
        this.feedforward = feedforward;
        this.nominalVoltage = nominalVoltage;
        this.homingCurrentAmps = homingCurrentAmps;
        this.homingSpeed = homingSpeed;
    }

    public static class Builder {
        private double velocityTolerance = 1;
        private double positionTolerance = 1;

        private double velocityConversion = 1;
        private double positionConversion = 1;

        private double minPosition = -999999999999.9;
        private double maxPosition = 999999999999.9;
        private double resetPosition = 0;

        private BiFunction<Double, Double, Double> feedforward = (pos, vel) -> 0.0;

        private double homingCurrentAmps = 50;
        private double homingSpeed = -0.2;

        private double nominalVoltage = 12.0;

        public ServoConstants build() {
            return new ServoConstants(
                    velocityTolerance,
                    positionTolerance,
                    velocityConversion,
                    positionConversion,
                    minPosition,
                    maxPosition,
                    resetPosition,
                    feedforward,
                    nominalVoltage,
                    homingCurrentAmps,
                    homingSpeed);
        }

        public Builder withVelocityTolerance(double velocityTolerance) {
            this.velocityTolerance = velocityTolerance;
            return this;
        }

        public Builder withPositionTolerance(double positionTolerance) {
            this.positionTolerance = positionTolerance;
            return this;
        }

        public Builder withVelocityConversion(double velocityConversion) {
            this.velocityConversion = velocityConversion;
            return this;
        }

        public Builder withPositionConversion(double positionConversion) {
            this.positionConversion = positionConversion;
            return this;
        }

        public Builder withMaxPosition(double maxPosition) {
            this.maxPosition = maxPosition;
            return this;
        }

        public Builder withMinPosition(double minPosition) {
            this.minPosition = minPosition;
            return this;
        }

        public Builder withResetPosition(double resetPosition) {
            this.resetPosition = resetPosition;
            return this;
        }

        public Builder withSimpleMotorFeedforward(BiFunction<Double, Double, Double> ff) {
            this.feedforward = ff;
            return this;
        }

        public Builder withHomingCurrentAmps(double homingCurrentAmps) {
            this.homingCurrentAmps = homingCurrentAmps;
            return this;
        }

        public Builder withHomingSpeed(double homingSpeed) {
            this.homingSpeed = homingSpeed;
            return this;
        }

        public Builder withNominalVoltage(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            return this;
        }
    }
}

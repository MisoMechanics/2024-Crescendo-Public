package team9442.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public abstract class SparkmaxSubsystem implements PeriodicSubsystem, Logged {
    private final CANSparkMax master;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;
    private final ServoConstants constants;
    private final List<CANSparkMax> followers = new ArrayList<>();
    protected final double kDt;

    private PeriodicIO periodicIO = new PeriodicIO();

    public SparkmaxSubsystem(CANSparkMax master, ServoConstants constants, double kDt) {
        this.master = master;
        this.kDt = kDt;
        this.constants = constants;
        this.encoder = master.getEncoder();
        this.controller = master.getPIDController();
    }

    public static class PeriodicIO {
        // Inputs
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double timestamp = 0;
        public double busVoltage = 0;
        public double appliedOutput;

        // Outputs
        public ControlType controlMode = ControlType.kDutyCycle;
        public double demand = 0;
        public double feedforward = 0;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = encoder.getPosition() * constants.positionConversion;
        periodicIO.velocity = encoder.getVelocity() * constants.velocityConversion;
        periodicIO.current = master.getOutputCurrent();
        periodicIO.timestamp = Timer.getFPGATimestamp();
        periodicIO.busVoltage = master.getBusVoltage();
        periodicIO.appliedOutput = master.getAppliedOutput();

        this.log("position", periodicIO.position);
        this.log("velocity", periodicIO.velocity);
        this.log("current", periodicIO.current);
        this.log("busVoltage", periodicIO.busVoltage);
        this.log("appliedOutput", periodicIO.appliedOutput);
        this.log("demand", periodicIO.demand);
    }

    @Override
    public void writePeriodicOutputs() {
        if (periodicIO.controlMode == ControlType.kDutyCycle) {
            master.set(periodicIO.demand);
        } else {
            controller.setReference(
                    periodicIO.demand,
                    periodicIO.controlMode,
                    0,
                    periodicIO.feedforward,
                    ArbFFUnits.kVoltage);
        }
    }

    public Command openloop(DoubleSupplier openloop) {
        return this.run(() -> setOpenloop(openloop.getAsDouble()))
                .finallyDo(() -> this.setOpenloop(0));
    }

    public Command openloop(double openloop) {
        return openloop(() -> openloop);
    }

    public Command voltage(DoubleSupplier voltage) {
        return this.run(() -> setVolts(voltage.getAsDouble()));
    }

    public Command voltage(double voltage) {
        return voltage(() -> voltage);
    }

    public Command currentBasedHome() {
        final var currentThresholdBreached =
                new Trigger(() -> getMasterCurrent() > constants.homingCurrentAmps);
        return parallel(
                openloop(constants.homingSpeed)
                        .raceWith(sequence(waitSeconds(0.5), waitUntil(currentThresholdBreached))));
    }

    @Override
    public void end() {
        setOpenloop(0);
    }

    public void setOpenloop(double output) {
        periodicIO.controlMode = ControlType.kDutyCycle;
        if (getPosition() > constants.maxPosition) {
            output = Math.min(0, output);
        } else if (getPosition() < constants.minPosition) {
            output = Math.max(0, output);
        }
        periodicIO.demand = output;
        periodicIO.feedforward = 0;
    }

    public void setVolts(double volts) {
        periodicIO.controlMode = ControlType.kVoltage;
        periodicIO.demand = volts;
        periodicIO.feedforward = 0;
    }

    /**
     * Raw PID (not motion magic)
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPosition(double position, double feedforward) {
        periodicIO.controlMode = ControlType.kPosition;
        periodicIO.feedforward = feedforward;
        periodicIO.demand =
                MathUtil.clamp(position, constants.minPosition, constants.maxPosition)
                        / constants.positionConversion;
    }

    /**
     * Motion Magic position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionMotionMagic(double position, double feedforward) {
        periodicIO.controlMode = ControlType.kSmartMotion;
        periodicIO.feedforward = feedforward;
        periodicIO.demand =
                MathUtil.clamp(position, constants.minPosition, constants.maxPosition)
                        / constants.positionConversion;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocity(double velocity, double feedforward) {
        periodicIO.controlMode = ControlType.kVelocity;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = velocity / constants.velocityConversion;
    }

    /** Sets all motors to brake mode */
    public void setToBrake() {
        setNeutralMode(IdleMode.kBrake);
    }

    /** Sets all motors to coast mode */
    public void setToCoast() {
        setNeutralMode(IdleMode.kCoast);
    }

    /**
     * Sets master and all followers to the mode
     *
     * @param mode either Brake or Coast
     */
    public void setNeutralMode(IdleMode mode) {
        master.setIdleMode(mode);
        for (CANSparkMax follower : followers) {
            follower.setIdleMode(mode);
        }
    }

    /** Sets the selected sensor to default reset position from constants */
    public void resetEncoder() {
        setEncoder(constants.resetPosition);
    }

    /**
     * sets the selected sensor to position
     *
     * @param position position in output units
     */
    protected REVLibError setEncoder(double position) {
        return encoder.setPosition(position / constants.positionConversion);
    }

    /**
     * @return the volts in the output units
     */
    public double getWantedDemand() {
        return periodicIO.demand;
    }

    /**
     * @return the volts in the output units
     */
    public double getBusVoltage() {
        return periodicIO.busVoltage;
    }

    /**
     * @return the volts in the output units
     */
    public double getAppliedOutput() {
        return periodicIO.appliedOutput;
    }

    /**
     * @return the velocity in the output units
     */
    public double getVelocity() {
        return periodicIO.velocity;
    }

    /**
     * @return ths position in the output units
     */
    public double getPosition() {
        return periodicIO.position;
    }

    public boolean atPositionSetpoint(double setpoint, double tolerance) {
        return Math.abs(getPosition() - setpoint) < tolerance;
    }

    public boolean atPositionSetpoint(double tolerance) {
        return atPositionSetpoint(periodicIO.demand * constants.positionConversion, tolerance);
    }

    public boolean atPositionSetpoint() {
        return atPositionSetpoint(constants.positionTolerance);
    }

    public boolean atVelocitySetpoint(double setpoint, double tolerance) {
        return Math.abs(getVelocity() - setpoint) < tolerance;
    }

    public boolean atVelocitySetpoint(double tolerance) {
        return atVelocitySetpoint(periodicIO.demand * constants.velocityConversion, tolerance);
    }

    public boolean atVelocitySetpoint() {
        return atVelocitySetpoint(constants.velocityTolerance);
    }

    /**
     * @return the timestamp for the position and velocity measurements
     */
    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public double getMasterCurrent() {
        return periodicIO.current;
    }

    protected REVLibError addFollower(CANSparkMax follower, boolean invertFromMaster) {
        final var error = follower.follow(master, invertFromMaster);
        if (REVLibError.kOk == error) {
            followers.add(follower);
        }
        return error;
    }
}

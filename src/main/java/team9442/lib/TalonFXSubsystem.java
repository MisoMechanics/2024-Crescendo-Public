package team9442.lib;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public abstract class TalonFXSubsystem implements PeriodicSubsystem, Logged {

    private final TalonFX master;
    private final List<TalonFX> followers = new ArrayList<>();
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    private final MotionMagicVoltage motionMagicDutyCycle = new MotionMagicVoltage(0);
    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private Follower masterOutput;
    private final ServoConstants constants;
    protected final double kDt;
    public static final int kLongStatusTimeMS = 255;
    public static final int kTimeoutMS = 100;

    private PeriodicIO periodicIO = new PeriodicIO();

    protected TalonFXSubsystem(TalonFX master, ServoConstants constants, double kDt) {
        this.master = master;
        this.kDt = kDt;
        this.constants = constants;
        this.master.clearStickyFaults(kLongStatusTimeMS);
    }

    public static class PeriodicIO {
        // Inputs
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double timestamp = 0;
        public double masterCurrent = 0;
        public double nativePosition = 0;
        public double appliedOutput = 0;

        // Outputs
        public ControlRequest controlMode = new EmptyControl();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.nativePosition = master.getRotorPosition().getValueAsDouble();
        periodicIO.position =
                master.getPosition().getValueAsDouble() * constants.positionConversion;
        periodicIO.velocity =
                master.getVelocity().getValueAsDouble() * constants.velocityConversion;
        periodicIO.current = master.getStatorCurrent().getValueAsDouble();
        periodicIO.timestamp = Timer.getFPGATimestamp();
        periodicIO.masterCurrent = master.getSupplyCurrent().getValueAsDouble();
        periodicIO.appliedOutput = master.getDutyCycle().getValueAsDouble();

        this.log("position", periodicIO.position);
        this.log("velocity", periodicIO.velocity);
        this.log("current", periodicIO.current);
        this.log("appliedOutput", periodicIO.appliedOutput);
        this.log("position demand", motionMagicDutyCycle.Position);
        this.log("velocity demand", velocityDutyCycle.Velocity);
    }

    @Override
    public void writePeriodicOutputs() {
        master.setControl(periodicIO.controlMode);
        if (masterOutput != null) {
            followers.forEach(follower -> follower.setControl(masterOutput));
        }
    }

    @Override
    public void end() {
        setOpenloop(0);
    }

    public Command openloop(DoubleSupplier openloop) {
        return this.run(() -> setOpenloop(openloop.getAsDouble()));
    }

    public Command openloop(double openloop) {
        return openloop(() -> openloop);
    }

    public Command currentBasedHome() {
        final var currentThresholdBreached =
                new Trigger(() -> getMasterCurrent() > constants.homingCurrentAmps);
        final var removeSoftLimit =
                runOnce(
                        () ->
                                this.master
                                        .getConfigurator()
                                        .apply(
                                                new SoftwareLimitSwitchConfigs()
                                                        .withReverseSoftLimitEnable(false)));
        final var addSoftLimit =
                runOnce(
                        () ->
                                this.master
                                        .getConfigurator()
                                        .apply(
                                                new SoftwareLimitSwitchConfigs()
                                                        .withReverseSoftLimitEnable(true)));

        return sequence(
                removeSoftLimit,
                parallel(
                        openloop(constants.homingSpeed)
                                .raceWith(
                                        sequence(
                                                waitSeconds(1),
                                                waitUntil(
                                                        currentThresholdBreached.and(
                                                                () ->
                                                                        Math.abs(getVelocity())
                                                                                < 1))))),
                runOnce(this::resetEncoder),
                addSoftLimit);
    }

    protected void setOpenloop(double output) {
        periodicIO.controlMode = dutyCycle;
        dutyCycle.Output = output;
    }

    /**
     * Raw PID (not motion magic)
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPosition(double position, double feedforward) {
        periodicIO.controlMode = positionDutyCycle;
        positionDutyCycle.FeedForward = feedforward / constants.nominalVoltage;
        positionDutyCycle.Position = position / constants.positionConversion;
    }

    protected void setPositionNative(double position, double feedforward) {
        periodicIO.controlMode = positionDutyCycle;
        positionDutyCycle.FeedForward = feedforward / constants.nominalVoltage;
        positionDutyCycle.Position = position;
    }

    /**
     * Motion Magic position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionMotionMagic(double position, double feedforward) {
        periodicIO.controlMode = motionMagicDutyCycle;
        motionMagicDutyCycle.Slot = 0;
        motionMagicDutyCycle.FeedForward = feedforward / constants.nominalVoltage;
        motionMagicDutyCycle.Position = position / constants.positionConversion;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocity(double velocity, double feedforward) {
        periodicIO.controlMode = velocityDutyCycle;
        velocityDutyCycle.FeedForward = feedforward / constants.nominalVoltage;
        velocityDutyCycle.Velocity = velocity / constants.velocityConversion;
    }

    protected void setControl(ControlRequest request) {
        periodicIO.controlMode = request;
    }

    /** Sets all motors to brake mode */
    public void setToBrake() {
        setNeutralMode(NeutralModeValue.Brake);
    }

    /** Sets all motors to coast mode */
    public void setToCoast() {
        setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets master and all followers to the mode
     *
     * @param mode either Brake or Coast
     */
    public void setNeutralMode(NeutralModeValue mode) {
        TalonFXConfigurator masterConfigurator = master.getConfigurator();
        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfigurator.refresh(masterConfiguration);
        masterConfiguration.MotorOutput.NeutralMode = mode;
        masterConfigurator.apply(masterConfiguration);
    }

    public void enableCurrentLimit() {
        TalonFXConfigurator masterConfigurator = master.getConfigurator();
        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfigurator.apply(masterConfiguration);
    }

    /** Sets the selected sensor to resetPosition (default) */
    public void resetEncoder() {
        master.setPosition(constants.resetPosition / constants.positionConversion);
    }

    /** Sets the selected sensor to resetPosition (default) */
    public void resetEncoder(int timeoutMS) {
        master.setPosition(constants.resetPosition / constants.positionConversion, timeoutMS);
    }

    /**
     * sets the selected sensor to position
     *
     * @param position position in output units
     */
    protected void setEncoder(double position) {
        master.setPosition(position / constants.positionConversion);
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

    /**
     * @return the timestamp for the position and velocity measurements
     */
    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public double getMasterCurrent() {
        return periodicIO.masterCurrent;
    }

    public double getNativePos() {
        return periodicIO.nativePosition;
    }

    public boolean atPositionSetpoint(double setpoint, double tolerance) {
        return Math.abs(getPosition() - setpoint) < tolerance;
    }

    public boolean atPositionSetpoint(double tolerance) {
        return atPositionSetpoint(
                positionDutyCycle.Position * constants.positionConversion, tolerance);
    }

    public boolean atPositionSetpoint() {
        return atPositionSetpoint(constants.positionTolerance);
    }

    public boolean atVelocitySetpoint(double setpoint, double tolerance) {
        return Math.abs(getVelocity() - setpoint) < tolerance;
    }

    public boolean atVelocitySetpoint(double tolerance) {
        return atVelocitySetpoint(
                velocityDutyCycle.Velocity * constants.velocityConversion, tolerance);
    }

    public boolean atVelocitySetpoint() {
        return atVelocitySetpoint(constants.velocityTolerance);
    }

    // this is bad. this will work with one follower but will not work if there are multiple
    // followers that do not all have the same direction and output

    protected void addFollower(TalonFX follower, boolean opposeMaster) {
        this.masterOutput = new Follower(this.master.getDeviceID(), opposeMaster);
        followers.add(follower);
    }
}

package team9442.frc2024.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import team9442.lib.ServoConstants;
import team9442.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {
    private final BiFunction<Double, Double, Double> feedforward;
    private final SysIdRoutine pivotSydIdRoutine;
    private final TorqueCurrentFOC characterizationControl = new TorqueCurrentFOC(0);

    public Pivot(TalonFX master, ServoConstants constants, double kDt) {
        super(master, constants, kDt);
        this.feedforward = constants.feedforward;
        pivotSydIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                edu.wpi.first.units.Units.Volts.of(4)
                                        .per(edu.wpi.first.units.Units.Seconds.of(1)),
                                edu.wpi.first.units.Units.Volts.of(25),
                                edu.wpi.first.units.Units.Seconds.of(6),
                                state -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts ->
                                        this.setControl(
                                                characterizationControl.withOutput(
                                                        volts.in(edu.wpi.first.units.Units.Volts))),
                                null,
                                this));
    }

    public Command runSysIdDynamic(Direction direction) {
        return pivotSydIdRoutine.dynamic(direction);
    }

    public Command runSysIdQuasistatic(Direction direction) {
        return pivotSydIdRoutine.quasistatic(direction);
    }

    public Command angle(DoubleSupplier angleSup) {
        return this.run(
                () -> {
                    final var angleDeg = angleSup.getAsDouble();
                    setPositionMotionMagic(
                            angleDeg, feedforward.apply(Units.degreesToRadians(angleDeg), 0.0));
                });
    }

    public Command holdAtCall() {
        Subsystem pivot = this;
        return new Command() {
            private double holdPosition = 250.0;
            Set<Subsystem> requirements = Set.of(pivot);

            @Override
            public void initialize() {
                holdPosition = getPosition();
            }

            @Override
            public void execute() {
                setPositionMotionMagic(holdPosition, kDt);
            }

            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    public Command angle(double angle) {
        return this.angle(() -> angle);
    }

    public Command angleWithEnd(double angle) {
        return this.angle(() -> angle).until(this::atPositionSetpoint);
    }
}

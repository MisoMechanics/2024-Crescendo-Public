package team9442.frc2024.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import team9442.lib.ServoConstants;
import team9442.lib.TalonFXSubsystem;

public class Rollers extends TalonFXSubsystem {
    private final BiFunction<Double, Double, Double> feedforward;
    private final SysIdRoutine rollersSysIdRoutine;
    private final TorqueCurrentFOC characterizationControl = new TorqueCurrentFOC(0);

    public Rollers(TalonFX master, ServoConstants constants, double kDt) {
        super(master, constants, kDt);
        feedforward = constants.feedforward;
        rollersSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.of(10).per(Units.Seconds.of(1)),
                                Units.Volts.of(60),
                                Units.Seconds.of(6),
                                state -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts ->
                                        this.setControl(
                                                characterizationControl.withOutput(
                                                        volts.in(Units.Volts))),
                                null,
                                this));
    }

    public Command velocity(DoubleSupplier surfaceVelocitySup) {
        return this.run(
                        () -> {
                            var surfaceVelocity = surfaceVelocitySup.getAsDouble();
                            setVelocity(surfaceVelocity, feedforward.apply(surfaceVelocity, 0.0));
                        })
                .finallyDo(() -> setOpenloop(0));
    }

    public Command runSysIdDynamic(Direction direction) {
        return rollersSysIdRoutine.dynamic(direction);
    }

    public Command runSysIdQuasistatic(Direction direction) {
        return rollersSysIdRoutine.quasistatic(direction);
    }

    public Command velocity(double surfaceVelocity) {
        return velocity(() -> surfaceVelocity);
    }

    public Rollers withFollower(TalonFX follower, boolean invertFromMaster) {
        super.addFollower(follower, invertFromMaster);
        return this;
    }
}

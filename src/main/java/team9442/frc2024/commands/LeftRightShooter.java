package team9442.frc2024.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team9442.frc2024.subsystems.Rollers;

public class LeftRightShooter {
    public final Rollers portSide;
    public final Rollers starboardSide;
    private final double ratio;
    // 0.875

    public LeftRightShooter(Rollers portSide, Rollers starboardSide, double ratio) {
        this.portSide = portSide;
        this.starboardSide = starboardSide;
        this.ratio = ratio;
    }

    public Command openloopZero() {
        return openloopEquals(0);
    }

    public Command openloopEquals(double demand) {
        return parallel(portSide.openloop(demand), starboardSide.openloop(demand));
    }

    /*
     * port goes the passed speed, starboard speed * ratio
     */
    public Command openloopPortControl(DoubleSupplier higher) {
        return parallel(
                portSide.openloop(higher),
                starboardSide.openloop(() -> higher.getAsDouble() * ratio));
    }

    /*
     * starboard goes the passed speed, port speed * ratio
     */
    public Command openloopStarboardControl(DoubleSupplier higher) {
        return parallel(
                starboardSide.openloop(higher),
                portSide.openloop(() -> higher.getAsDouble() * ratio));
    }

    public Command velocity(DoubleSupplier higher) {
        return parallel(
                portSide.velocity(higher),
                starboardSide.velocity(() -> higher.getAsDouble() * ratio));
    }

    public boolean bothAtVelocitySetpoint() {
        return portSide.atVelocitySetpoint() && starboardSide.atVelocitySetpoint();
    }

    public double getVelocityAverage() {
        return (portSide.getVelocity() + starboardSide.getVelocity()) / 2.0;
    }
}

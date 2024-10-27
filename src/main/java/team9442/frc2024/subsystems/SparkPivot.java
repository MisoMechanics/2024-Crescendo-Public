package team9442.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import team9442.lib.ServoConstants;
import team9442.lib.SparkmaxSubsystem;

public class SparkPivot extends SparkmaxSubsystem {
    private final BiFunction<Double, Double, Double> feedforward;

    public SparkPivot(CANSparkMax master, ServoConstants constants, double kDt) {
        super(master, constants, kDt);
        this.feedforward = constants.feedforward;
    }

    public Command angle(DoubleSupplier angleSup) {
        return this.run(
                () -> {
                    final var angleDeg = angleSup.getAsDouble();
                    setPosition(angleDeg, feedforward.apply(Units.degreesToRadians(angleDeg), 0.0));
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
                setPosition(holdPosition, kDt);
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

    public SparkPivot withFollower(CANSparkMax follower, boolean invertFromMaster) {
        super.addFollower(follower, invertFromMaster);
        return this;
    }
}

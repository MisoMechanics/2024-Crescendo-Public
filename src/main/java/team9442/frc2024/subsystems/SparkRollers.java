package team9442.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import team9442.lib.ServoConstants;
import team9442.lib.SparkmaxSubsystem;

public class SparkRollers extends SparkmaxSubsystem {
    private final BiFunction<Double, Double, Double> feedforward;

    public SparkRollers(CANSparkMax master, ServoConstants constants, double kDt) {
        super(master, constants, kDt);
        feedforward = constants.feedforward;
    }

    public Command velocity(DoubleSupplier surfaceVelocitySup) {
        return this.run(
                () -> {
                    var surfaceVelocity = surfaceVelocitySup.getAsDouble();
                    setVelocity(surfaceVelocity, feedforward.apply(surfaceVelocity, 0.0));
                });
    }

    public Command velocity(double surfaceVelocity) {
        return velocity(() -> surfaceVelocity);
    }

    public SparkRollers withFollower(CANSparkMax follower, boolean invertFromMaster) {
        super.addFollower(follower, invertFromMaster);
        return this;
    }
}

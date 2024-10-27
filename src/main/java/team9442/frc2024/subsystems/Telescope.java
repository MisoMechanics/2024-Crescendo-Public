package team9442.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team9442.lib.ServoConstants;
import team9442.lib.SparkmaxSubsystem;

public class Telescope extends SparkmaxSubsystem {

    public Telescope(CANSparkMax master, ServoConstants constants, double kDt) {
        super(master, constants, kDt);
    }

    Command length(DoubleSupplier length) {
        return this.run(() -> setPosition(length.getAsDouble(), 0));
    }

    Command length(double length) {
        return length(() -> length);
    }

    public Telescope withFollower(CANSparkMax follower, boolean invertFromMaster) {
        super.addFollower(follower, invertFromMaster);
        return this;
    }
}

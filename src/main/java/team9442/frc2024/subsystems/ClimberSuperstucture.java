package team9442.frc2024.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import team9442.frc2024.constants.TelescopeConstants;

public class ClimberSuperstucture extends SubsystemBase {
    private final Telescope right;
    private final Telescope left;

    public ClimberSuperstucture(Telescope right, Telescope left) {
        this.right = right;
        this.left = left;

        TelescopeConstants.kLeft.setInverted(true);
        TelescopeConstants.kRight.setSmartCurrentLimit(40);
        TelescopeConstants.kLeft.setSmartCurrentLimit(40);

        right.setToBrake();
        left.setToBrake();
    }

    public Command openLoopBothWithEnd(DoubleSupplier openLoop) {
        return Commands.parallel(right.openloop(openLoop), left.openloop(openLoop));
    }

    public Command openLoopRightWithEnd(DoubleSupplier openLoop) {
        return right.openloop(openLoop).finallyDo(() -> right.openloop(0));
    }

    public Command openLoopLeftWithEnd(DoubleSupplier openLoop) {
        return left.openloop(openLoop).finallyDo(() -> left.openloop(0));
    }
}

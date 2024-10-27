package team9442.frc2024.util;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BiConsumer;
import team9442.lib.CommandsUtil;

public class RumbleControl {
    private final BiConsumer<RumbleType, Double> rumbleJoystick;
    public final Runnable stopRumble;

    public RumbleControl(BiConsumer<RumbleType, Double> rumbleJoystick) {
        this.rumbleJoystick = rumbleJoystick;
        this.stopRumble = () -> rumbleJoystick.accept(RumbleType.kBothRumble, 0.0);
    }

    public void addTrigger(Trigger trigger, RumbleType type, double intensity, double seconds) {
        trigger.onTrue(
                CommandsUtil.repeatTimes(
                        sequence(
                                runOnce(() -> rumbleJoystick.accept(type, intensity)),
                                waitSeconds(0.6),
                                runOnce(stopRumble)),
                        3));
    }

    public Command rumble(double intensity, double seconds) {
        return startEnd(() -> rumbleJoystick.accept(RumbleType.kBothRumble, intensity), stopRumble)
                .withTimeout(seconds);
    }
}

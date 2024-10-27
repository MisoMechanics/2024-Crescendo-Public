package team9442.frc2024.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;

    private final AddressableLEDBuffer black;

    public LEDs(AddressableLED leds, AddressableLEDBuffer black) {
        this.leds = leds;
        this.black = black;
        this.leds.start();
    }

    public void setBuffer(AddressableLEDBuffer buffer) {
        this.leds.setData(buffer);
    }

    public Command solid(AddressableLEDBuffer buffer) {
        return this.runOnce(() -> this.leds.setData(buffer)).repeatedly();
    }

    public Command solidOnce(AddressableLEDBuffer buffer) {
        return this.runOnce(() -> this.leds.setData(buffer));
    }

    public Command flash(AddressableLEDBuffer buffer) {
        return sequence(solidOnce(buffer), waitSeconds(0.1), solidOnce(black), waitSeconds(0.1))
                .repeatedly();
    }
}

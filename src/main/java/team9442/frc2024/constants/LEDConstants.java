package team9442.frc2024.constants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final AddressableLED kLEDs = new AddressableLED(0);
    public static final AddressableLEDBuffer kWhiteBuffer = new AddressableLEDBuffer(200);
    public static final AddressableLEDBuffer kRedBuffer = new AddressableLEDBuffer(200);
    public static final AddressableLEDBuffer kGreenBuffer = new AddressableLEDBuffer(200);
    public static final AddressableLEDBuffer kBlackBuffer = new AddressableLEDBuffer(200);
    public static final AddressableLEDBuffer kYellowBuffer = new AddressableLEDBuffer(200);
    public static final AddressableLEDBuffer kPurpleBuffer = new AddressableLEDBuffer(200);
    public static final AddressableLEDBuffer kOrangeBuffer = new AddressableLEDBuffer(200);

    static {
        kLEDs.setLength(200);

        for (int i = 0; i < kWhiteBuffer.getLength(); ++i) {
            kWhiteBuffer.setLED(i, Color.kWhite);
            kRedBuffer.setLED(i, Color.kRed);
            kGreenBuffer.setLED(i, Color.kGreen);
            kBlackBuffer.setLED(i, Color.kBlack);
            kYellowBuffer.setLED(i, Color.kYellow);
            kPurpleBuffer.setLED(i, Color.kPurple);
            kOrangeBuffer.setLED(i, Color.kOrange);
        }
    }
}

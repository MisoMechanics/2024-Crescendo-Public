package team9442.frc2024.constants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import team9442.lib.ServoConstants;

public class TelescopeConstants {
    public static final CANSparkMax kRight =
            new CANSparkMax(GlobalConstants.TelescopeIds.KPortSideId, MotorType.kBrushless);

    public static final CANSparkMax kLeft =
            new CANSparkMax(GlobalConstants.TelescopeIds.kStarboardSideId, MotorType.kBrushless);

    public static final ServoConstants kConstants =
            new ServoConstants.Builder()
                    .withResetPosition(0)
                    .withMinPosition(3)
                    .withMaxPosition(105)
                    .build();

    public static final double climberSpeed = 1;
}

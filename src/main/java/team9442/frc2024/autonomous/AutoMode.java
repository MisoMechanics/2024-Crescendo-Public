package team9442.frc2024.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoMode {
    public final String name;
    public final Command command;
    public final Pose2d pose;
    public final Runnable init;

    public AutoMode(String name, Command command, Pose2d pose) {
        this(name, command, pose, () -> {});
    }

    public AutoMode(String name, Command command, Pose2d pose, Runnable init) {
        this.name = name;
        this.command = command;
        this.pose = pose;
        this.init = init;
    }
}

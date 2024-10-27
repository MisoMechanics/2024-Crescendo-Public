package team9442.frc2024.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.function.Consumer;
import team9442.frc2024.constants.FieldConstants;
import team9442.lib.AllianceUpdatedObserver;

public final class AutoChooser extends SendableChooser<AutoMode>
        implements AllianceUpdatedObserver {
    private boolean flipPose = false;

    private final Consumer<Pose2d> resetPose;

    public AutoChooser(AutoCommands autoCommands, Consumer<Pose2d> resetPose) {
        super();
        this.resetPose = resetPose;
        onChange(this::autoModeChosen);
        for (final AutoMode mode : autoCommands) {
            this.addOption(mode.name, mode);
        }
    }

    public void onAllianceFound(Alliance alliance) {
        flipPose = alliance == Alliance.Red;
        final var selected = getSelected();
        if (selected == null) {
            return;
        }
        autoModeChosen(selected);
    }

    public void autoModeChosen(AutoMode mode) {
        mode.init.run();
        this.resetPose.accept(flipPose ? FieldConstants.flipped(mode.pose) : mode.pose);
    }
}

package team9442.frc2024.alignment;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Logged;
import team9442.frc2024.vision.NoteDetectionPV;

public class NoteTracking implements Logged {
    private final PIDController profiledThetaController;
    private final PIDController profiledXController;
    private final PIDController profiledYController;
    private final NoteDetectionPV noteTracker;

    private boolean enabled = true;

    public NoteTracking(
            PIDController profiledThetaController,
            PIDController profiledXController,
            PIDController profiledYController,
            NoteDetectionPV noteTracker) {
        this.profiledThetaController = profiledThetaController;
        this.profiledXController = profiledXController;
        this.profiledYController = profiledYController;
        profiledThetaController.enableContinuousInput(-180, 180);
        this.noteTracker = noteTracker;
    }

    public double findRotationVelocity() {
        if (!noteTracker.hasTargets() || !enabled) {
            return 0;
        }
        final var output = profiledThetaController.calculate(noteTracker.getTx(), 0);
        this.log("note detection output", output);
        return output;
    }

    public double findXVelocity() {
        final var output = profiledXController.calculate(noteTracker.getTx(), 0);
        this.log("note detection x vel output", output);
        return output;
    }

    public double findYVelocity() {
        final var output = Math.max(profiledYController.calculate(noteTracker.getTx(), 0), 1);
        this.log("note detection y vel output", output);
        return output;
    }

    public void enable() {
        this.enabled = true;
    }

    public void disable() {
        this.enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public Command enableC() {
        return Commands.runOnce(this::enable);
    }

    public Command disableC() {
        return Commands.runOnce(this::disable);
    }

    public boolean atNote() {
        return noteTracker.hasTargets();
    }
}

package team9442.frc2024.vision;

import org.photonvision.PhotonCamera;

public class NoteDetectionPV {

    private final PhotonCamera noteCam;
    private final PeriodicIO periodicIO = new PeriodicIO();

    public static class PeriodicIO {
        public double tx = 0;
        public double ty = 0;
        public boolean hasTarget = false;
    }

    public NoteDetectionPV(PhotonCamera noteCam) {
        this.noteCam = noteCam;
    }

    public double getTx() {
        final var result = this.noteCam.getLatestResult();

        if (!result.hasTargets()) {
            return 0;
        }
        return result.getBestTarget().getYaw();
    }

    public double getTy() {
        final var result = this.noteCam.getLatestResult();

        if (!result.hasTargets()) {
            return 0;
        }
        return result.getBestTarget().getPitch();
    }

    public boolean hasTargets() {
        return this.noteCam.getLatestResult().hasTargets();
    }
}

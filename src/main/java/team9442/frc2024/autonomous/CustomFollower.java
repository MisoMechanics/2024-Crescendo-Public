package team9442.frc2024.autonomous;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import team9442.frc2024.alignment.NoteTracking;

public class CustomFollower implements Consumer<ChassisSpeeds>, Logged {

    public static enum FollowerState {
        FOLLOW_PATH,
        AIM_SPEAKER,
        AIM_NOTE
    }

    private final Consumer<SwerveRequest> dtSetRequest;
    private final DoubleSupplier rotationalAimVelocityRadiansPerSecond;
    private FollowerState state = FollowerState.FOLLOW_PATH;
    private final NoteTracking noteTracking;

    private final ApplyChassisSpeeds autoFollower =
            new ApplyChassisSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    public CustomFollower(
            Consumer<SwerveRequest> dtSetRequest,
            NoteTracking noteTracking,
            DoubleSupplier rotationalAimVelocityRadiansPerSecond) {
        this.dtSetRequest = dtSetRequest;
        this.noteTracking = noteTracking;
        this.rotationalAimVelocityRadiansPerSecond = rotationalAimVelocityRadiansPerSecond;
    }

    public void setFollowPath() {
        this.state = FollowerState.FOLLOW_PATH;
        this.log("state", this.state.toString());
    }

    public Command followPath() {
        return runOnce(this::setFollowPath);
    }

    public void setAimSpeaker() {
        this.state = FollowerState.AIM_SPEAKER;
        this.log("state", this.state.toString());
    }

    public void setAimNote() {
        this.state = FollowerState.AIM_SPEAKER;
        this.log("state", this.state.toString());
    }

    public Command aimNote() {
        return runOnce(this::setAimNote);
    }

    public Command aimSpeaker() {
        return runOnce(this::setAimSpeaker);
    }

    public void accept(ChassisSpeeds robotRelativeSpeeds) {
        if (state == FollowerState.AIM_NOTE) {
            //            robotRelativeSpeeds = new ChassisSpeeds(0, 3,
            // noteTracking.findRotationVelocity());
            //            robotRelativeSpeeds = new ChassisSpeeds(noteTracking.findXVelocity(),
            // noteTracking.findYVelocity(), noteTracking.findRotationVelocity());
            robotRelativeSpeeds =
                    new ChassisSpeeds(
                            noteTracking.findYVelocity(),
                            noteTracking.findXVelocity(),
                            noteTracking.findRotationVelocity());
        }
        if (state == FollowerState.AIM_SPEAKER) {
            robotRelativeSpeeds =
                    new ChassisSpeeds(
                            robotRelativeSpeeds.vxMetersPerSecond,
                            robotRelativeSpeeds.vyMetersPerSecond,
                            rotationalAimVelocityRadiansPerSecond.getAsDouble());
        }
        dtSetRequest.accept(autoFollower.withSpeeds(robotRelativeSpeeds));
    }
}

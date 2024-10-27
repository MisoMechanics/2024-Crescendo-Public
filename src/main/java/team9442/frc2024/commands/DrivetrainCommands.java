package team9442.frc2024.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import team9442.frc2024.subsystems.Drivetrain;
import team9442.frc2024.vision.NoteDetectionPV;

public class DrivetrainCommands implements Logged {

    private SwerveRequest.FieldCentric fieldCentricControl =
            new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private SwerveRequest.RobotCentric robotCentricControl =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Command drive(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier zRot,
            DoubleSupplier translationScaleFactor,
            DoubleSupplier rotationalAimVelocity,
            BooleanSupplier allowAutoAim,
            DoubleSupplier ampAimVelocity,
            BooleanSupplier allowAmpAim,
            DoubleSupplier noteAimVelocity,
            BooleanSupplier allowNoteTracking,
            NoteDetectionPV noteDetectionPV,
            BooleanSupplier fieldOriented,
            BooleanSupplier isOpenLoop) {

        return Commands.run(
                () -> {
                    double xVel = xSpeed.getAsDouble() * drivetrain.getMaxSpeedMpS();
                    double yVel = ySpeed.getAsDouble() * drivetrain.getMaxSpeedMpS();
                    double zRotRads = zRot.getAsDouble() * drivetrain.getMaxRotationRadpS();
                    final double aimVelocityDegrees = rotationalAimVelocity.getAsDouble();
                    if (allowAmpAim.getAsBoolean()) {
                        zRotRads = Units.degreesToRadians(ampAimVelocity.getAsDouble());
                    }
                    if (allowAutoAim.getAsBoolean() && aimVelocityDegrees != 0) {
                        zRotRads = Units.degreesToRadians(aimVelocityDegrees);
                    }

                    if (allowNoteTracking.getAsBoolean() && noteDetectionPV != null) {
                        zRotRads += Units.degreesToRadians(noteAimVelocity.getAsDouble());
                        double intakingScale =
                                MathUtil.clamp((noteDetectionPV.getTy() + 25) / 50, 0.3, 1);
                        log("scale for intaking", intakingScale);
                        xVel = Math.sqrt(Math.pow(xVel, 2) + Math.pow(yVel, 2)) * intakingScale;
                        yVel = 0;
                    }

                    this.log("wanted x", xVel);
                    this.log("wanted y", yVel);
                    this.log("wanted z", zRotRads);
                    this.log("aim velocity degrees", aimVelocityDegrees);
                    fieldCentricControl.ForwardReference = ForwardReference.OperatorPerspective;
                    fieldCentricControl
                            .withVelocityX(xVel)
                            .withVelocityY(yVel)
                            .withRotationalRate(zRotRads);
                    robotCentricControl
                            .withVelocityX(xVel)
                            .withVelocityY(yVel)
                            .withRotationalRate(zRotRads);

                    drivetrain.setRequest(
                            allowNoteTracking.getAsBoolean()
                                    ? robotCentricControl
                                    : fieldCentricControl);
                },
                drivetrain);
    }

    public Command rotate(DoubleSupplier rotMultiplier) {
        return this.drive(
                () -> 0,
                () -> 0,
                rotMultiplier,
                () -> 0,
                () -> 0,
                () -> false,
                () -> 0,
                () -> false,
                () -> 0,
                () -> false,
                null,
                () -> true,
                () -> true);
    }

    public DrivetrainCommands(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    private final Drivetrain drivetrain;
}

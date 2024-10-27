// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package team9442.frc2024.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import team9442.frc2024.subsystems.Drivetrain;

public class WheelRadiusCharacterization extends Command implements Logged {
    private static double characterizationSpeed = 0.1;
    private static final double driveRadius = Units.inchesToMeters(14.672465);
    private final DoubleSupplier gyroYawRadsSupplier;

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;

        Direction(int value) {
            this.value = value;
        }
    }

    private final Drivetrain drive;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    private SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric();

    public WheelRadiusCharacterization(Drivetrain drive, Direction omegaDirection) {
        this.drive = drive;
        this.omegaDirection = omegaDirection;
        addRequirements(drive);
        this.gyroYawRadsSupplier = () -> Units.degreesToRadians(drive.getHeading());
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {

        // Run drive at velocity

        drive.setRequest(
                swerveRequest.withRotationalRate(characterizationSpeed * omegaDirection.value));

        // Get yaw and wheel positions
        accumGyroYawRads +=
                MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
        SmartDashboard.putNumber("DrivePosition", averageWheelPosition);
        SmartDashboard.putNumber("AccumGyroYawRads", accumGyroYawRads);
        SmartDashboard.putNumber(
                "CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
        characterizationSpeed += 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            SmartDashboard.putNumber(
                    "Effective Wheel Radius: ", Units.metersToInches(currentEffectiveWheelRadius));
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team9442.frc2024.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Monologue;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private boolean previousWasNothing = true;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        m_robotContainer.drivetrain.setToBrake();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Monologue.setFileOnly(DriverStation.isFMSAttached());
        Monologue.updateAll();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.drivetrain.setToBrake();
    }

    @Override
    public void disabledPeriodic() {
        if (previousWasNothing) {
            m_robotContainer.allianceChecker.periodic();
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.drivetrain.setToBrake();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        previousWasNothing = false;
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        m_robotContainer.drivetrain.setToBrake();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        previousWasNothing = false;
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.getTestCommand().schedule();
        previousWasNothing = false;
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}

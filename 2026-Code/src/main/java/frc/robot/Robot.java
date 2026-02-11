// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AlphaSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Subsystems2026;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private AlphaRobotContainer m_alphaRobotContainer;

    private final boolean isAlpha() {
        return true;
    }

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        if (isAlpha()) {
            m_alphaRobotContainer = new AlphaRobotContainer();
        } else {
            m_robotContainer = new RobotContainer();
        }
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(false, 10);

        if (!Constants.GlobalConstants.RED_ALLIANCE.isPresent() && DriverStation.getAlliance().isPresent()) {
            Constants.GlobalConstants.RED_ALLIANCE = Optional
                    .of(DriverStation.getAlliance().get().equals(Alliance.Red));
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (isAlpha()) {
            m_autonomousCommand = m_alphaRobotContainer.getAutonomousCommand();
            if (m_autonomousCommand != null) {
                CommandScheduler.getInstance().schedule(m_autonomousCommand);
            }
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        if (isAlpha()) {
            AlphaSubsystem.hubState.update();
        } else {
            Subsystems2026.hubState.update();
        }
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}

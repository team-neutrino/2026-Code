// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Subsystem;

import static frc.robot.util.Subsystem.*;

public class RobotContainer {
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandXboxController m_buttonController = new CommandXboxController(1);
  Subsystem m_subsystemContainer = new Subsystem();
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    shooter.setDefaultCommand(shooter.defaultCommand());
    intake.setDefaultCommand(intake.defaultCommand());
  }

  private void configureBindings() {
    m_buttonController.a().whileTrue(shooter.runShooter());
    m_buttonController.x().whileTrue(intake.runIntake());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

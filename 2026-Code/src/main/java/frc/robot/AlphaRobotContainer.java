// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.command_factories.*;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.SplineToPoint;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AlphaSubsystem;
import frc.robot.util.Constants.DriveToPointConstants.TargetMode;

import static frc.robot.util.AlphaSubsystem.*;

public class AlphaRobotContainer {
  private CommandXboxController m_buttonController = new CommandXboxController(1);
  private AlphaSubsystem m_subsystemContainer;

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public AlphaRobotContainer() {
    m_subsystemContainer = new AlphaSubsystem();
    configureDefaultCommands();
    configureBindings();
    configureNamedCommands();
    swerve.registerTelemetry(logger::telemeterize);
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDefaultCommands() {
    alphaShooter.setDefaultCommand(alphaShooter.defaultCommand());
    alphaIntake.setDefaultCommand(alphaIntake.defaultCommand());
    alphaKicker.setDefaultCommand(alphaKicker.defaultCommand());
    alphabotVision.setDefaultCommand(alphabotVision.AlphabotVisionDefaultCommand());
    swerve.setDefaultCommand(
        // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with
            // negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        swerve.applyRequest(() -> idle).ignoringDisable(true));
  }

  private void configureBindings() {
    m_buttonController.y().whileTrue(alphaShooter.runShooter());
    m_buttonController.a().whileTrue(alphaKicker.runKicker());
    m_buttonController.x().whileTrue(alphaIntake.runIntake());
    m_buttonController.b().whileTrue(alphaIntake.runOuttake());

    m_driverController.start().whileTrue(swerve.resetYaw());
    m_driverController.x().whileTrue(new SplineToPoint(m_driverController, TargetMode.SHOOTING));
    m_driverController.y().whileTrue(new SplineToPoint(m_driverController, TargetMode.SHUTTLING));
    // uncomment on real robot
    // m_driverController.leftBumper()
    // .whileTrue(new SplineToPoint(m_driverController, TargetMode.CLIMBING));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("AlphaDeployIntake", IntakeFactory.deployIntake());
    NamedCommands.registerCommand("AlphaRetractIntake", IntakeFactory.retractIntake());
    NamedCommands.registerCommand("AlphaRunIntake", IntakeFactory.runIntake());
    NamedCommands.registerCommand("AlphaRunOuttake", IntakeFactory.runOuttake());
    NamedCommands.registerCommand("AlphaRunIndexer", IndexFactory.runSpindexer());
    NamedCommands.registerCommand("AlphaAutonClimb", ClimbFactory.autoClimb());
    NamedCommands.registerCommand("AlphaDriveToPoint", new DriveToPoint());
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

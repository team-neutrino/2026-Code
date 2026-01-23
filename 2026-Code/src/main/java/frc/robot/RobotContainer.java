// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AlphaSubsystem;

import static frc.robot.util.AlphaSubsystem.*;

public class RobotContainer {
  private CommandXboxController m_buttonController = new CommandXboxController(1);
  private AlphaSubsystem m_subsystemContainer;
  // private Subsystem m_subsystemContainer = new Subsystem();
  // comment out whichever subsystem container you're not testing

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CommandXboxController joystick = new CommandXboxController(0);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public RobotContainer() {
    m_subsystemContainer = new AlphaSubsystem();
    configureDefaultCommands();
    configureBindings();
    swerve.registerTelemetry(logger::telemeterize);
  }

  private void configureDefaultCommands() {
    alphaShooter.setDefaultCommand(alphaShooter.defaultCommand());
    alphaIntake.setDefaultCommand(alphaIntake.defaultCommand());
    alphabotLimelight.setDefaultCommand(alphabotLimelight.AlphaLimelightDefaultCommand());
    alphaKicker.setDefaultCommand(alphaKicker.defaultCommand());
    swerve.setDefaultCommand(
        // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
    joystick.start().whileTrue(swerve.resetYaw());
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

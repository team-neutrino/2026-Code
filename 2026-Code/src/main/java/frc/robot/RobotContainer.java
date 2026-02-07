
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.command_factories.ClimbFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Subsystems;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.Subsystems.*;

import java.util.function.BooleanSupplier;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private Subsystems m_subsystemContainer;

  public RobotContainer() {
    m_subsystemContainer = new Subsystems();
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    climb.setDefaultCommand(climb.defaultClimbCommand());
    shooter.setDefaultCommand(shooter.defaultCommand());
    intake.setDefaultCommand(intake.defaultCommand());
    index.setDefaultCommand(index.defaultCommand());
    kicker.setDefaultCommand(kicker.defaultCommand());
    swerve.setDefaultCommand(swerve.swerveDefaultCommand(m_driverController));
  }

  private void configureBindings() {
    m_buttonController.rightTrigger().whileTrue(ClimbFactory.lowerClimbArm()); // Random buttons subject to change
    m_buttonController.b().whileTrue(IntakeFactory.deployAndRunIntake());
    m_buttonController.a().whileTrue(shooter.resetHood());
    m_buttonController.leftTrigger().whileTrue(IntakeFactory.deployIntake());

    m_driverController.start().whileTrue(swerve.resetYaw());
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

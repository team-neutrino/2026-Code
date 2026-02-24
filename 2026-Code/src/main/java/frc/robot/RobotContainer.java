
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.command_factories.SuperstructureFactory;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.Subsystems.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private Subsystems m_subsystemContainer;

  public RobotContainer() {
    m_subsystemContainer = new Subsystems();
    configureDefaultCommands();
    configureBindings();
    swerve.registerTelemetry(logger::telemeterize);

  }

  private void configureDefaultCommands() {
    shooter.setDefaultCommand(shooter.defaultCommand());
    intake.setDefaultCommand(intake.defaultCommand());
    index.setDefaultCommand(index.defaultCommand());
    swerve.setDefaultCommand(swerve.swerveDefaultCommand(m_driverController));
    turret.setDefaultCommand(turret.defaultCommand());
  }

  private void configureBindings() {
    m_driverController.start().whileTrue(swerve.resetYaw());

    m_buttonController.a().onTrue(shooter.resetHood());
    m_buttonController.b().whileTrue(SuperstructureFactory.Shuttle());
    m_buttonController.x().whileTrue(index.feedShooter());
    m_buttonController.y().whileTrue(SuperstructureFactory.spitFuel());
    m_buttonController.leftTrigger().toggleOnTrue(IntakeFactory.deployAndRunIntake());
    m_buttonController.rightTrigger().toggleOnTrue(IntakeFactory.deployAndRunOuttake());

  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

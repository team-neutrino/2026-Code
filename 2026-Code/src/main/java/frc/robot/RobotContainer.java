
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.ClimbFactory;
import frc.robot.command_factories.ShooterFactory;
import frc.robot.util.Subsystems2026;
import static frc.robot.util.Constants.ShooterConstants.*;

import static frc.robot.util.Subsystems2026.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);

  private Subsystems2026 m_subsystemContainer;

  public RobotContainer() {
    m_subsystemContainer = new Subsystems2026();
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    climb.setDefaultCommand(climb.defaultClimbCommand());
    shooter.setDefaultCommand(shooter.defaultCommand());
    intake.setDefaultCommand(intake.defaultCommand());
    index.setDefaultCommand(index.defaultCommand());
  }

  private void configureBindings() {
    m_buttonController.leftBumper().whileTrue(ClimbFactory.climbUp()); // Random buttons that are subject to change
    m_buttonController.rightBumper().whileTrue(ClimbFactory.climbDown()); // Random buttons that are subject to change
    m_buttonController.povUp().whileTrue(ShooterFactory.shootFromInterpolationTable(1.2));
    m_buttonController.povDown().whileTrue(ShooterFactory.shootFromInterpolationTable(3.9));
    m_buttonController.povLeft().whileTrue(ShooterFactory.shootFromInterpolationTable(6.7)); // : )
    m_buttonController.povRight().whileTrue(ShooterFactory.shootFromInterpolationTable(8.8));
    m_buttonController.a().whileTrue(ShooterFactory.shootFromInterpolationTable(11.1));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}


package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.ClimbFactory;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
    m_buttonController.povUp().whileTrue(ShooterFactory.shootingAngleFromFixedPosition(fakeEnum.RADIAL_CLOSE));
    m_buttonController.povDown().whileTrue(ShooterFactory.shootingAngleFromFixedPosition(fakeEnum.RADIAL_FAR));
    m_buttonController.povLeft().whileTrue(ShooterFactory.shootingAngleFromFixedPosition(fakeEnum.WALL));
    m_buttonController.povRight().whileTrue(ShooterFactory.shootingAngleFromFixedPosition(fakeEnum.DEPOT));
    m_buttonController.a().whileTrue(ShooterFactory.shootingAngleFromFixedPosition(fakeEnum.OUTPOST));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

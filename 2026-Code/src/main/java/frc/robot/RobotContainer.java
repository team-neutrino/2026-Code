
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.ClimbFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.command_factories.ShooterFactory;
import frc.robot.util.Subsystems;
import static frc.robot.util.Constants.ShooterConstants.*;

import static frc.robot.util.Subsystems.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);

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
  }

  private void configureBindings() {
    m_buttonController.rightTrigger().whileTrue(ClimbFactory.lowerClimbArm()); // Random buttons subject to change
    m_buttonController.b().whileTrue(IntakeFactory.deployAndRunIntake());
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

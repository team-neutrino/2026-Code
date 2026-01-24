
package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.Subsystems2026;

import static frc.robot.util.Subsystems2026.*;

public class RobotContainer {
    private CommandXboxController m_buttonController = new CommandXboxController(1);
    private Subsystems2026 m_subsystemContainer = new Subsystems2026();

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        m_subsystemContainer = new Subsystems2026();
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}

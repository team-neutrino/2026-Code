
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.command_factories.SuperstructureFactory;
import frc.robot.commands.DriveToPoint;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Subsystems;
import frc.robot.util.Constants.AutonConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.Constants.DriveToPointConstants.SHOOT_POSES;
import static frc.robot.util.Subsystems.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private Command m_autonPath;

  private Subsystems m_subsystemContainer;

  public RobotContainer() {
    m_subsystemContainer = new Subsystems();
    configureDefaultCommands();
    configureBindings();
    configureNamedCommands();
    m_autonPath = new PathPlannerAuto(AutonConstants.CURRENT_AUTON);
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
    m_buttonController.b().whileTrue(SuperstructureFactory.shuttle());
    m_buttonController.x().whileTrue(index.feedShooter());
    m_buttonController.y().whileTrue(SuperstructureFactory.spitFuel());
    m_buttonController.leftTrigger().toggleOnTrue(IntakeFactory.deployAndRunIntake());
    m_buttonController.rightTrigger().toggleOnTrue(IntakeFactory.deployAndRunOuttake());

  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("DriveToPointFinite", SuperstructureFactory.DriveToPointFinite(SHOOT_POSES));
    NamedCommands.registerCommand("DriveToPoint", new DriveToPoint(SHOOT_POSES));
    NamedCommands.registerCommand("deployAndRunIntake", IntakeFactory.deployAndRunIntake());
    NamedCommands.registerCommand("runIntake", IntakeFactory.runIntake());
    NamedCommands.registerCommand("feedShooter", SuperstructureFactory.FeedShooter());
    NamedCommands.registerCommand("noShoot", SuperstructureFactory.noShooting());
  }

  public Command getAutonomousCommand() {
    Command auto;

    if (Subsystems.swerve == null) {
      return new InstantCommand();
    }
    try {
      auto = m_autonPath;
    } catch (Exception e) {
      // DO NOT CHANGE THE CODE IN THIS CATCH BLOCK
      System.err.println("Caught exception when loading auto");
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }
}

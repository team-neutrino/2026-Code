
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.command_factories.SuperstructureFactory;
import frc.robot.commands.DriveToPoint;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.Constants.DriveToPointConstants.NEUTRAL_ZONE_POSES;
import static frc.robot.util.Constants.DriveToPointConstants.SHOOT_POSES;
import static frc.robot.util.Subsystems.*;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private SendableChooser<Command> m_chooser;

  private Subsystems m_subsystemContainer;

  public RobotContainer() {
    m_subsystemContainer = new Subsystems();
    configureDefaultCommands();
    configureBindings();
    configureNamedCommands();

    swerve.registerTelemetry(logger::telemeterize);

    List<String> autonNames = AutoBuilder.getAllAutoNames();
    for (String auton : autonNames) {
      AutoBuilder.buildAuto(auton);
    }

    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", m_chooser);
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
    m_driverController.leftTrigger().whileTrue(swerve.slowSwerveDrive(m_driverController));
    m_driverController.rightTrigger().whileTrue(swerve.slowestSwerveDrive(m_driverController));

    m_buttonController.a().onTrue(shooter.resetHood());
    m_buttonController.b().whileTrue(SuperstructureFactory.shuttle());
    m_buttonController.x().whileTrue(index.feedShooter());
    m_buttonController.y().whileTrue(index.noKickAndSpin());
    m_buttonController.povUp().toggleOnTrue(IntakeFactory.shakeHopper());
    m_buttonController.leftTrigger().onTrue(IntakeFactory.toggleIntake());
    m_buttonController.rightTrigger().whileTrue(SuperstructureFactory.shootAndReverseKicker());
    m_buttonController.leftBumper().whileTrue(IntakeFactory.runIntake());
    m_buttonController.rightBumper().whileTrue(IntakeFactory.runOuttake());
    m_buttonController.povRight().whileTrue(SuperstructureFactory.programmedShot());
    m_buttonController.back().whileTrue(shooter.unstickYakit());
    m_buttonController.povDown().whileTrue(IntakeFactory.deployAndRunIntake());

  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("DriveToPointFinite", SuperstructureFactory.DriveToPointFinite(NEUTRAL_ZONE_POSES));
    NamedCommands.registerCommand("DriveToPoint", new DriveToPoint(NEUTRAL_ZONE_POSES, true));
    NamedCommands.registerCommand("deployAndRunIntake", IntakeFactory.deployAndRunIntake());
    NamedCommands.registerCommand("retractIntake", intake.retractIntake());
    NamedCommands.registerCommand("feedShooter", index.autonDefaultCommand());
    NamedCommands.registerCommand("noShoot", index.noKickAndSpin());
    NamedCommands.registerCommand("shooterDefault", shooter.autonDefaultCommand());
    NamedCommands.registerCommand("noDrive", swerve.noDrive());
    NamedCommands.registerCommand("Unbeach", swerve.unbeach());
    NamedCommands.registerCommand("shakeHopper", IntakeFactory.shakeHopper().repeatedly());
  }

  public Command getAutonomousCommand() {
    Command auto;

    if (Subsystems.swerve == null) {
      return new InstantCommand();
    }
    try {
      auto = m_chooser.getSelected();
    } catch (Exception e) {
      // DO NOT CHANGE THE CODE IN THIS CATCH BLOCK
      System.err.println("Caught exception when loading auto");
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }
}

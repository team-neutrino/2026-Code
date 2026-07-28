
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.command_factories.SuperstructureFactory;
import frc.robot.commands.DriveToPoint;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.Constants.DriveToPointConstants.NEUTRAL_ZONE_POSES;
import static frc.robot.util.Constants.DriveToPointConstants.SHOOT_POSES;
import static frc.robot.util.Subsystems.*;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private AutoFactory autoFactory;
  private AutoChooser autoChooser;

  private Subsystems m_subsystemContainer;

  public RobotContainer() {
    m_subsystemContainer = new Subsystems();
    configureDefaultCommands();
    configureBindings();
    configureNamedCommands();

    autoChooser = new AutoChooser();
    autoFactory = new AutoFactory(swerve::getCurrentPose, swerve::resetPose, swerve::followChoreoTrajectory, true, swerve);

    swerve.registerTelemetry(logger::telemeterize);
    autoChooser.addRoutine("Example", () -> example());
    autoFactory.bind("Marker", intake.silly());
    SmartDashboard.putData("AutoChooser", autoChooser);

    
  }

  private void configureDefaultCommands() {
    shooter.setDefaultCommand(shooter.defaultCommand());
    intake.setDefaultCommand(intake.defaultCommand());
    index.setDefaultCommand(index.defaultCommand());
    swerve.setDefaultCommand(swerve.swerveDefaultCommand(m_driverController));
    turret.setDefaultCommand(turret.defaultCommand());
  }

  public AutoRoutine example() {
    AutoRoutine routine = autoFactory.newRoutine("New Path");
    AutoTrajectory neutral = routine.trajectory("NeutralLeft");
    AutoTrajectory depot = routine.trajectory("Depot");
    routine.active().onTrue(
      Commands.sequence(
        neutral.resetOdometry(),
        neutral.cmd(),
        Commands.race(
          intake.silly(),
          Commands.waitSeconds(2)
        ),
        depot.cmd()
      )
    );
    
    return routine;
  }

  private void configureBindings() {
    m_driverController.start().whileTrue(swerve.resetYaw());
    m_driverController.leftTrigger().whileTrue(swerve.slowSwerveDrive(m_driverController));
    m_driverController.rightTrigger().whileTrue(swerve.slowestSwerveDrive(m_driverController));
    m_driverController.rightBumper().whileTrue(index.noKickAndSpin());

    m_buttonController.a().onTrue(shooter.resetHood());
    m_buttonController.b().whileTrue(SuperstructureFactory.shuttle());
    m_buttonController.x().whileTrue(index.feedShooter());
    m_buttonController.y().whileTrue(index.noKickAndSpin());
    m_buttonController.povUp().toggleOnTrue(IntakeFactory.shakeHopper());
    m_buttonController.leftTrigger().onTrue(IntakeFactory.toggleIntake());
    m_buttonController.rightTrigger().whileTrue(SuperstructureFactory.shootAndReverseKicker());
    m_buttonController.leftBumper().whileTrue(IntakeFactory.runIntake());
    m_buttonController.rightBumper().whileTrue(IntakeFactory.runOuttake());
    m_buttonController.povRight().whileTrue(SuperstructureFactory.outpostProgrammedShot());
    m_buttonController.povLeft().whileTrue(SuperstructureFactory.depotProgrammedShot());
    m_buttonController.back().whileTrue(SuperstructureFactory.unstickYakit());
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
    NamedCommands.registerCommand("shakeHopper", IntakeFactory.autonShakeHopper().repeatedly());
  }

  public Command getAutonomousCommand() {
    Command auto;

    if (Subsystems.swerve == null) {
      return new InstantCommand();
    }
    try {
      auto = autoChooser.selectedCommandScheduler();
    } catch (Exception e) {
      // DO NOT CHANGE THE CODE IN THIS CATCH BLOCK
      System.err.println("Caught exception when loading auto");
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }

  public void teleopInit() {
    intake.setTeleopCurrentLimit();
  }
}

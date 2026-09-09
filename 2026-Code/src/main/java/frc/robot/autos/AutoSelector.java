package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.Subsystems;

public class AutoSelector {
  private AutoSelector m_autoChooser = new AutoSelector();

  public AutoSelector() {
    m_autoChooser.addRoutine("Example", () -> example());

    SmartDashboard.putData("AutoChooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    Command auto;

    if (Subsystems.swerve == null) {
      return new InstantCommand();
    }
    try {
      auto = m_autoChooser.selectedCommandScheduler();
    } catch (Exception e) {
      // DO NOT CHANGE THE CODE IN THIS CATCH BLOCK
      System.err.println("Caught exception when loading auto");
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }
}

package frc.robot.autos.trajectories;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autos.AutoBase;
import frc.robot.command_factories.IntakeFactory;

import static frc.robot.util.Subsystems.*;

public class Nothing {
    public static AutoRoutine example() {
        AutoRoutine routine = AutoBase.autoFactory.newRoutine("New Path");
        routine.active().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                swerve.noDrive(),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand()
                        )
                        ));

        return routine;
    }
}

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

public class LeftDepot {
    public static AutoRoutine example() {
        AutoRoutine routine = AutoBase.autoFactory.newRoutine("New Path");
        AutoTrajectory neutral = routine.trajectory("NeutralLeftAll");
        AutoTrajectory depot = routine.trajectory("Depot");
        routine.active().onTrue(
                Commands.sequence(
                        neutral.resetOdometry(),
                        Commands.race(
                              neutral.cmd(),
                              index.noKickAndSpin(),
                              Commands.sequence(
                                Commands.waitSeconds(0.7),
                                IntakeFactory.deployAndRunIntake()
                              )
                        ),
                        swerve.unbeach(),
                        Commands.race(
                                swerve.noDrive(),
                                IntakeFactory.shakeHopper(),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand(),
                                Commands.waitSeconds(2)),
                        Commands.race(
                                depot.cmd(),
                                IntakeFactory.deployAndRunIntake(),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand())
                        ));

        return routine;
    }
}

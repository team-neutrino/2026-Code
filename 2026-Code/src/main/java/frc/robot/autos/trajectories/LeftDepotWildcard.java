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

public class LeftDepotWildcard {
    public static AutoRoutine example() {
        AutoRoutine routine = AutoBase.autoFactory.newRoutine("New Path");
        AutoTrajectory neutral = routine.trajectory("NeutralLeftAllWild");
        AutoTrajectory depot1 = routine.trajectory("Depot1");
        AutoTrajectory depot2 = routine.trajectory("Depot2");
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
                                IntakeFactory.autonShakeHopper().repeatedly(),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand(),
                                Commands.waitSeconds(1.5)),
                        Commands.race(
                                depot1.cmd(),
                                Commands.sequence(
                                        Commands.race(
                                                Commands.waitSeconds(3),
                                                IntakeFactory.autonShakeHopper().repeatedly()
                                        ),
                                        IntakeFactory.deployAndRunIntake()
                                ),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand()),
                        Commands.race(
                                depot2.cmd(),
                                IntakeFactory.autonShakeHopper(),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand()
                        ),
                        Commands.race(
                                swerve.noDrive(),
                                IntakeFactory.autonShakeHopper(),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand()
                        )
                        ));

        return routine;
    }
}

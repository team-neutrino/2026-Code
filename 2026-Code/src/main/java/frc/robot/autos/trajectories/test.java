package frc.robot.autos.trajectories;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoBase;

import static frc.robot.util.Subsystems.*;

public class test {
    public static AutoRoutine example() {
        AutoRoutine routine = AutoBase.autoFactory.newRoutine("New Path");
        AutoTrajectory neutral = routine.trajectory("NeutralLeft");
        AutoTrajectory depot = routine.trajectory("Depot");
        routine.active().onTrue(
                Commands.sequence(
                        neutral.resetOdometry(),
                        neutral.cmd(),
                        swerve.unbeach(),
                        Commands.race(
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand(),
                                Commands.waitSeconds(2)),
                        Commands.race(
                                depot.cmd(),
                                index.autonDefaultCommand(),
                                shooter.autonDefaultCommand()),
                        swerve.noDrive()));

        return routine;
    }
}

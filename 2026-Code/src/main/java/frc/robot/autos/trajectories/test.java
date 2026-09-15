package frc.robot.autos.trajectories;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoBase;

public class test {
    public static AutoRoutine example() {
        AutoRoutine routine = AutoBase.autoFactory.newRoutine("New Path");
        AutoTrajectory neutral = routine.trajectory("NeutralLeft");
        AutoTrajectory depot = routine.trajectory("Depot");
        routine.active().onTrue(
                Commands.sequence(
                        neutral.resetOdometry(),
                        neutral.cmd(),
                        Commands.race(
                                Commands.waitSeconds(2)),
                        depot.cmd()));

        return routine;
    }
}

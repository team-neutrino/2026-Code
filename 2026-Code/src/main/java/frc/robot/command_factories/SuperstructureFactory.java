// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SplineToPoint;
import frc.robot.util.Constants.DriveToPointConstants.TargetMode;

import frc.robot.util.Constants.*;

import static frc.robot.util.Subsystems.*;

/** Add your docs here. */
public class SuperstructureFactory {
    public static Command climbRunCommand(CommandXboxController driverController) {
        if (!(climb.getClimbTargetPosition() <= ClimbConstants.CLIMB_POSITION + ClimbConstants.ALLOWED_ERROR
                && climb.getClimbTargetPosition() >= ClimbConstants.CLIMB_POSITION - ClimbConstants.ALLOWED_ERROR)) {
            return new ParallelCommandGroup(new SplineToPoint(driverController, TargetMode.CLIMBING),
                    climb.defaultClimbCommand().until(() -> climb.isCANRangeDetected())
                            .andThen(ClimbFactory.climbSequentialCommand()));
        } else {
            return ClimbFactory.climbReleaseCommand();
        }
    }
}

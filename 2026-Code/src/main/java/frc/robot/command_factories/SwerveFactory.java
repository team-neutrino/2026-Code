// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.SplineToPoint;

import static frc.robot.util.AlphaSubsystem.*;

/** Add your docs here. */
public class SwerveFactory {
    public static Command splineThenDTP() {
        SplineToPoint spline = new SplineToPoint();
        DriveToPoint dtp = new DriveToPoint();
        return spline.andThen(swerve.PrintCommand());
    }
}

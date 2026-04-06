// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Constants.ShooterConstants.*;
import static frc.robot.util.Constants.TurretConstants.*;
import static frc.robot.util.Subsystems.*;

import frc.robot.commands.DriveToPoint;

/** Add your docs here. */
public class SuperstructureFactory {

    public static Command shootOnly() {
        return index.noKickAndSpin().alongWith(shooter.shootingSpeed(6000));
    }

    public static Command shuttle() {
        return index.shuttle().alongWith(shooter.shuttle());
    }

    public static Command spitFuel() {
        return index.feedShooter()
                .alongWith(shooter.runShooterAndHood(SOFT_SHOT_SPEED, SOFT_SHOT_ANGLE));
    }

    public static Command noShooting() {
        return index.noKickAndSpin();
    }

    public static Command programmedShot() {
        return turret.setTargetAngleCommand(PROGRAMMED_SHOT_ANGLE).alongWith(shooter.programmedShot());
    }

    public static Command DriveToPointFinite(List<Pose2d> poses) {
        DriveToPoint drive = new DriveToPoint(poses, false);
        return drive.until(() -> Math.abs(swerve.getCurrentPose().getX() - drive.getTarget().getX()) <= .4
                && Math.abs(swerve.getCurrentPose().getY() - drive.getTarget().getY()) <= .4);
    }
}
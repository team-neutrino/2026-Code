// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Constants.ShooterConstants.SOFT_SHOT_ANGLE;
import static frc.robot.util.Constants.ShooterConstants.SOFT_SHOT_SPEED;
import static frc.robot.util.Subsystems.*;

/** Add your docs here. */
public class SuperstructureFactory {
    public static Command shuttle() {
        return (index.kickWhenPress().alongWith(shooter.shuttle()));
    }

    public static Command spitFuel() {
        return index.kickWhenPress().alongWith(index.spinWhenPress())
                .alongWith(shooter.runShooterAndHood(SOFT_SHOT_SPEED, SOFT_SHOT_ANGLE));
    }

}

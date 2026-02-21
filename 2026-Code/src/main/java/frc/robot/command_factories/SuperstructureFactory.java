// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.util.Subsystems.*;

/** Add your docs here. */
public class SuperstructureFactory {
    public static Command FeedShooter() {
        return (kicker.kickWhenPress()
                .alongWith(index.spinWhenPress()));
    }
}

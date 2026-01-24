// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.AlphaSubsystem;

import static frc.robot.util.Constants.DriveToPointConstants.*;

public class DriveToPoint extends PointControl {

    public DriveToPoint() {
        super();
        addRequirements(AlphaSubsystem.swerve);
    }

    private void drive() {
        double xVelocity = getPIDControl().getXVelocity(), yVelocity = getPIDControl().getYVelocity();

        xVelocity = MathUtil.clamp(xVelocity, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);
        yVelocity = MathUtil.clamp(yVelocity, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);

        AlphaSubsystem.swerve.setVelocity(xVelocity, yVelocity, getPIDControl().getRotation());
    }

    @Override
    public void initialize() {
        super.initialize();
        getPIDControl().setTarget(getTarget());
    }

    @Override
    public void execute() {
        super.execute();
        drive();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

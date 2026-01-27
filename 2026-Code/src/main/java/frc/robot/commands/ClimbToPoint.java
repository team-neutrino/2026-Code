package frc.robot.commands;

import static frc.robot.util.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.alpha_subsystems.AlphabotSwerve.SwerveRequestStash;

import static frc.robot.util.AlphaSubsystem.swerve;
import static frc.robot.util.Subsystems2026.climb;

import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;

public class ClimbToPoint extends Command {

    private double currentYVelocity = CLIMB_VELOCITY;
    private double currentXVelocity = 0;

    public ClimbToPoint() {

    }

    private void velocityDrive(double velx, double vely) {
        SwerveRequestStash.driveWithVelocity
                .withVelocityX(velx)
                .withVelocityY(vely);
        swerve.setControl(SwerveRequestStash.driveWithVelocity);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (RED_ALLIANCE.get()) {
            velocityDrive(currentXVelocity, currentYVelocity);
            if (currentYVelocity > 0 && climb.isCANRangeDetected()) {
                currentYVelocity = 0;
                currentXVelocity = CLIMB_VELOCITY;
            }
            if (currentXVelocity < 0 && climb.getCANRangeDistance() < DISTANCE_FROM_CLIMB) {
                currentXVelocity = 0;
            }
        } else {
            velocityDrive(currentXVelocity, currentYVelocity);
            if (currentYVelocity > 0 && climb.isCANRangeDetected()) {
                currentYVelocity = 0;
                currentXVelocity = -CLIMB_VELOCITY;
            }
            if (currentXVelocity < 0 && climb.getCANRangeDistance() < DISTANCE_FROM_CLIMB) {
                currentXVelocity = 0;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // set driving to point false
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

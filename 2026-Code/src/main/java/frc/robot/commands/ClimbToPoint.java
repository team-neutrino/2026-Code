package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.alpha_subsystems.AlphabotSwerve.SwerveRequestStash;

import static frc.robot.util.AlphaSubsystem.swerve;

public class ClimbToPoint extends Command {

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

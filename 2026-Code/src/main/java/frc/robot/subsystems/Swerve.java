package frc.robot.subsystems;

import frc.robot.generated.CommandSwerveDrivetrain;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.hardware.core.CorePigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.generated.TunerConstants;

import static frc.robot.util.Constants.SwerveConstants.*;


public class Swerve extends CommandSwerveDrivetrain{
    CorePigeon2 m_pigeon = new CorePigeon2(0);
    
    public Swerve() {
        super(TunerConstants.DrivetrainConstants, 
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

        getPigeon2().getConfigurator().apply(new GyroTrimConfigs().withGyroScalarZ(GYRO_SCALAR_Z));

        configureRequestPID();
        // if the robot power was never killed but code was redeployed/rebooted then the
        // swerve's yaw will zero itself but the pigeon will retain its previous value.
        resetRotation(Rotation2d.fromDegrees(getYawDegrees()));
    }
      
    public double getYaw360() {
        return getPigeon2().getYaw().getValueAsDouble() % 360;
    }
    
    public double getYawDegrees() {
        return Math.toDegrees(getYawRadians());
    }

    public double getYawRadians() {
        return MathUtil.angleModulus(Math.toRadians(getPigeon2().getYaw().getValueAsDouble()));
    }
    
    public Pose2d getCurrentPose() {
        return getState().Pose;
    }
    
    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    /**
     * Resets the yaw to 0, so the direction you're currently facing is the new
     * forwards.
     */
    public void resetYaw() {
        resetRotation(new Rotation2d(0));
        getPigeon2().reset();
        System.out.println("Yaw reset to 0");
        // need more research on the following
        // seedFieldCentric();
    }

    public void setControlAndApplyChassis(ChassisSpeeds speeds) {
        // setControl(
        //     SwerveRequestStash.autonDrive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
        //         .withRotationalRate(speeds.omegaRadiansPerSecond));
    }


    @Override
    public void periodic() {
        super.periodic();
    }

    public void configureRequestPID() {
        //SwerveRequestStash.driveWithVelocity.HeadingController.setPID(DRIVE_ASSIST_KP, 0, AUTO_ALIGN_D);
    }

    public class SwerveRequestStash {

    }

}

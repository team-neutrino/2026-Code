package frc.robot.alpha_subsystems;

import frc.robot.generated.CommandSwerveDrivetrain;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import static frc.robot.util.Constants.GlobalConstants.*;
import static frc.robot.util.Constants.SwerveConstants.*;

import java.io.IOException;

public class AlphabotSwerve extends CommandSwerveDrivetrain {

    private SlewRateLimiter m_slewLimit = new SlewRateLimiter(4, -Integer.MAX_VALUE, 0);

    public AlphabotSwerve() {
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

        configurePathPlanner();
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
    public Command resetYaw() {
        return run(() -> {
            resetRotation(new Rotation2d(0));
            getPigeon2().reset();
            System.out.println("Yaw reset to 0");
            // need more research on the following
            // seedFieldCentric();
        });
    }

    public void setControlAndApplyChassis(ChassisSpeeds speeds) {
        setControl(
                SwerveRequestStash.autonDrive.withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    private void configurePathPlanner() {
        double pTranslation = 1;
        double iTranslation = 0;
        double dTranslation = 0;
        double pRotation = 1;
        double iRotation = 0;
        double dRotation = 0;
        PIDConstants translationConstants = new PIDConstants(pTranslation, iTranslation, dTranslation);
        PIDConstants rotationConstants = new PIDConstants(pRotation, iRotation, dRotation);

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getCurrentPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::setControlAndApplyChassis,
                new PPHolonomicDriveController(
                        translationConstants,
                        rotationConstants),
                config,
                () -> {
                    return RED_ALLIANCE.isPresent() && RED_ALLIANCE.get();
                },
                this);
    }

    public Command swerveDefaultCommand(CommandXboxController joystick) {
        return run(() -> {
            double forward = -joystick.getLeftY();
            double left = -joystick.getLeftX();
            double rotation = -joystick.getRightX();

            setControl(SwerveRequestStash.drive
                    .withVelocityX(forward * MAX_SPEED)
                    .withVelocityY(left * MAX_SPEED)
                    .withRotationalRate(rotation * MAX_ROTATION_SPEED));
        });
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void configureRequestPID() {
        SwerveRequestStash.driveWithVelocity.HeadingController.setPID(DRIVE_ASSIST_KP, 0, AUTO_ALIGN_D);
    }

    public class SwerveRequestStash {
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(MAX_SPEED * 0.06)
                .withRotationalDeadband(MAX_ROTATION_SPEED * 0.06)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

        public static final SwerveRequest.RobotCentric autonDrive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.Velocity);

        public static final SwerveRequest.FieldCentricFacingAngle driveWithVelocity = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    }

}

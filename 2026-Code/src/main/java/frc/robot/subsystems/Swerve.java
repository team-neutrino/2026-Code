package frc.robot.subsystems;

import static frc.robot.util.Constants.FieldMeasurementConstants.*;

import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;
import static frc.robot.util.Constants.ShooterConstants.NOT_MOVING_THRESHOLD;
import static frc.robot.util.Constants.ShooterConstants.NOT_TURNING_THRESHOLD;
import static frc.robot.util.Constants.SwerveConstants.AUTO_ALIGN_D;
import static frc.robot.util.Constants.SwerveConstants.GYRO_SCALAR_Z;
import static frc.robot.util.Constants.SwerveConstants.MAX_ROTATION_SPEED;
import static frc.robot.util.Constants.SwerveConstants.MAX_SPEED;
import static frc.robot.util.Constants.SwerveConstants.ROTATIONAL_P;
import static frc.robot.util.Constants.TurretConstants.TURRET_OFFSET_X;
import static frc.robot.util.Constants.TurretConstants.TURRET_OFFSET_Y;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Constants.GlobalConstants;

public class Swerve extends CommandSwerveDrivetrain {

    private SlewRateLimiter m_slewLimit = new SlewRateLimiter(4, -Integer.MAX_VALUE, 0);

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

        configurePathPlanner();
    }

    public double getYaw360() {
        return getPigeon2().getYaw().getValueAsDouble() % 360;
    }

    public double getPitch() {
        return getPigeon2().getPitch().getValueAsDouble();
    }

    public double getYawDegrees() {
        return Math.toDegrees(getYawRadians());
    }

    public double getRoll() {
        return getPigeon2().getRoll().getValueAsDouble();
    }

    public double getYawRate() {
        return getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getPitchRate() {
        return getPigeon2().getAngularVelocityYWorld().getValueAsDouble();
    }

    public double getRollRate() {
        return getPigeon2().getAngularVelocityXWorld().getValueAsDouble();
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

    public void seedYawMT1(double MT1YawDegrees, double MT1Weight) {
        double pigeonWeight = 1 - MT1Weight;
        // double pigeonWeight = 1;
        MT1Weight = 0;
        double xAvg = (pigeonWeight * Math.cos(Math.toRadians(getYawDegrees())))
                + (MT1Weight * Math.cos(Math.toRadians(MT1YawDegrees)));
        double yAvg = (pigeonWeight * Math.sin(Math.toRadians(getYawDegrees())))
                + (MT1Weight * Math.sin(Math.toRadians(MT1YawDegrees)));
        double weightedAverage = Math.atan2(yAvg, xAvg);
        resetRotation(new Rotation2d(weightedAverage));
        getPigeon2().setYaw(Math.toDegrees(weightedAverage));
    }

    public void setVelocity(double xVelocity, double yVelocity, Rotation2d targetDirection) {
        SwerveRequestStash.driveWithVelocity
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withTargetDirection(targetDirection);
        setControl(SwerveRequestStash.driveWithVelocity);
    }

    public void setControlAndApplyChassis(ChassisSpeeds speeds) {
        setControl(
                SwerveRequestStash.autonDrive.withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public double getFromHubToTurret() {
        double robotX = getCurrentPose().getMeasureX().baseUnitMagnitude() + TURRET_OFFSET_Y;
        double robotY = getCurrentPose().getMeasureY().baseUnitMagnitude() + TURRET_OFFSET_X;

        if (!GlobalConstants.RED_ALLIANCE.isPresent()) {
            return 0;
        }

        Pose2d hubPose = GlobalConstants.RED_ALLIANCE.get() ? RED_HUB : BLUE_HUB;

        double hubDistanceX = hubPose.getX() - robotX;
        double hubDistanceY = hubPose.getY() - robotY;

        return Math.sqrt(Math.pow(hubDistanceX, 2) + Math.pow(hubDistanceY, 2));
    }

    public boolean inNeutralOrOpposingZone() {
        double robotX = getCurrentPose().getMeasureX().baseUnitMagnitude();

        if (GlobalConstants.RED_ALLIANCE.isPresent() && GlobalConstants.RED_ALLIANCE.get()) {
            return robotX < ALLIANCE_ZONE_RED;
        } else {
            return robotX > ALLIANCE_ZONE_BLUE;
        }
    }

    public double getSpeedMetersPerSecond() {
        return Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2)
                + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2));
    }

    public double getAngularSpeedDegreesPerSecond() {
        return Math.abs(Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));
    }

    public boolean isNotMovingOrTurning() {
        return getSpeedMetersPerSecond() < NOT_MOVING_THRESHOLD
                && getAngularSpeedDegreesPerSecond() < NOT_TURNING_THRESHOLD;
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
        } catch (IOException e) {
            System.out.println("Failed to instantiate RobotConfig config; IOException");
            e.printStackTrace();
        } catch (ParseException e) {
            System.out.println("Failed to instantiate RobotConfig config; ParseException");
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

    public Command noDrive() {
        return run(() -> {
            setControl(SwerveRequestStash.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        });
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void configureRequestPID() {
        SwerveRequestStash.driveWithVelocity.HeadingController.setPID(ROTATIONAL_P, 0, AUTO_ALIGN_D);
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

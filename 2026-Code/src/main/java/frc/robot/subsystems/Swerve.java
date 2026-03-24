package frc.robot.subsystems;

import static frc.robot.util.Subsystems.shooterArbiter;

import static frc.robot.util.Constants.FieldMeasurementConstants.*;

import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;
import static frc.robot.util.Constants.ShooterConstants.*;
import static frc.robot.util.Constants.SwerveConstants.*;
import static frc.robot.util.Constants.TurretConstants.*;
import static frc.robot.util.Constants.AutonConstants.*;

import java.io.IOException;
import java.util.Optional;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Constants.AutonConstants;
import frc.robot.util.Constants.GlobalConstants;
import frc.robot.util.Constants.ShooterConstants.shooterConditions;
import frc.robot.util.Subsystems;

public class Swerve extends CommandSwerveDrivetrain {

    private SlewRateLimiter m_slewLimit = new SlewRateLimiter(SLEW_LIMIT, -Integer.MAX_VALUE, 0);
    private boolean m_brakeEngaged = false;
    private Pose2d hubPose = Pose2d.kZero;
    private double m_turretTargetAngle = 0.0;
    private CommandXboxController m_driverController;
    private double joystickVx;
    private double joystickVy;

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

    public boolean isUpright() {
        return (Math.abs(getRoll()) > 180 - BEACHED_ANGLE && Math.abs(getPitch()) < BEACHED_ANGLE);
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

    public Translation2d getTurretGlobal() {
        Translation2d turretTranslation = new Translation2d(TURRET_OFFSET_FRONT, TURRET_OFFSET_SIDE);
        return getCurrentPose()
                .getTranslation()
                .plus(turretTranslation.rotateBy(new Rotation2d(getYawRadians())));
    }

    public double getFromHubToTurret() {
        if (!GlobalConstants.RED_ALLIANCE.isPresent()) {
            return 0;
        }
        return hubPose.getTranslation().getDistance(getTurretGlobal());
    }

    public boolean inNeutralOrOpposingZone() {
        double robotX = getCurrentPose().getMeasureX().baseUnitMagnitude();

        if (GlobalConstants.RED_ALLIANCE.isPresent() && GlobalConstants.RED_ALLIANCE.get()) {
            return robotX < ALLIANCE_ZONE_RED;
        } else {
            return robotX > ALLIANCE_ZONE_BLUE;
        }
    }

    public double getFieldRelativeTargetAngle() {
        return m_turretTargetAngle;
    }

    private double calculateFieldRelativeTargetAngle() {
        if (!GlobalConstants.RED_ALLIANCE.isPresent()) {
            return 0.0;
        }

        Pose2d robotPose = getCurrentPose();
        Translation2d turretGlobal = getTurretGlobal();
        double robotX = robotPose.getMeasureX().baseUnitMagnitude();
        double robotY = robotPose.getMeasureY().baseUnitMagnitude();
        Pose2d shuttlePose = GlobalConstants.RED_ALLIANCE.get()
                ? (robotY > MID_FIELD_Y ? SHUTTLE_TARGET_TOP_RED : SHUTTLE_TARGET_BOTTOM_RED)
                : (robotY > MID_FIELD_Y ? SHUTTLE_TARGET_TOP_BLUE : SHUTTLE_TARGET_BOTTOM_BLUE);

        boolean isInAllianceZone = (GlobalConstants.RED_ALLIANCE.get() && robotX >= ALLIANCE_ZONE_RED)
                || (!GlobalConstants.RED_ALLIANCE.get() && robotX <= ALLIANCE_ZONE_BLUE);

        Pose2d targetPose = isInAllianceZone ? hubPose : shuttlePose;
        double targetDistanceX = targetPose.getX() - (turretGlobal.getX());
        double targetDistanceY = targetPose.getY() - (turretGlobal.getY());

        return Math.toDegrees(Math.atan2(targetDistanceY, targetDistanceX));
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        Rotation2d angle = Rotation2d.fromDegrees(getYawDegrees());
        double vx = Math.abs(joystickVx) <= 0.342 ? 0
                : getChassisSpeeds().vxMetersPerSecond;
        double vy = Math.abs(joystickVy) <= 0.342 ? 0
                : getChassisSpeeds().vyMetersPerSecond;

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                vx,
                vy,
                getChassisSpeeds().omegaRadiansPerSecond,
                angle);
    }

    public Pose2d getHubPose() {
        Pose2d intialPose = BLUE_HUB;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            intialPose = (alliance.get() == Alliance.Blue)
                    ? BLUE_HUB
                    : RED_HUB;
        }
        Translation2d realHubLocation = intialPose.getTranslation();

        Translation2d currentTranslation = getTurretGlobal();
        ChassisSpeeds fieldSpeeds = getFieldRelativeChassisSpeeds();

        double currentDist = currentTranslation.getDistance(realHubLocation);
        double lookAheadTime = TIME_OF_FLIGHT.get(currentDist);

        Translation2d adjustedHub = realHubLocation;
        for (int i = 0; i < CONVERGENCE_ITERATIONS; i++) {
            double offsetX = realHubLocation.getX()
                    - (fieldSpeeds.vxMetersPerSecond * (lookAheadTime + TURRET_LATENCY));
            double offsetY = realHubLocation.getY()
                    - (fieldSpeeds.vyMetersPerSecond * (lookAheadTime + TURRET_LATENCY));
            adjustedHub = new Translation2d(offsetX, offsetY);
            currentDist = currentTranslation.getDistance(adjustedHub);
            lookAheadTime = TIME_OF_FLIGHT.get(currentDist);
        }

        return new Pose2d(adjustedHub, intialPose.getRotation());
    }

    public double getSpeedMetersPerSecond() {
        return Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2)
                + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2));
    }

    public double getAngularSpeedDegreesPerSecond() {
        return Math.abs(Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));
    }

    public boolean isNotMovingTooFastOrTurning() {
        return getSpeedMetersPerSecond() < SHOOT_WHILE_MOVING_THRESHOLD
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

    private void checkEngageBrake(double forward, double left, double rotation) {
        if (!m_brakeEngaged && Math.abs(forward) < BRAKE_ALLOWED_ERROR && Math.abs(left) < BRAKE_ALLOWED_ERROR
                && Math.abs(rotation) < BRAKE_ALLOWED_ERROR
                && getSpeedMetersPerSecond() < START_BRAKING_VELOCITY) {
            m_brakeEngaged = true;
        } else if (Math.abs(forward) > BRAKE_ALLOWED_ERROR || Math.abs(left) > BRAKE_ALLOWED_ERROR
                || Math.abs(rotation) > BRAKE_ALLOWED_ERROR) {
            m_brakeEngaged = false;
        }
    }

    public Command slowSwerveDrive(CommandXboxController joystick) {
        return run(() -> {
            double forward = -joystick.getLeftY();
            double left = -joystick.getLeftX();
            double rotation = -joystick.getRightX();
            double magnitude = Math.hypot(forward, left) * (SLOW_MAX_SPEED);
            magnitude = m_slewLimit.calculate(magnitude);
            checkEngageBrake(forward, left, rotation);

            if (m_brakeEngaged) {
                setControl(SwerveRequestStash.brake);
            } else {
                setControl(SwerveRequestStash.drive
                        .withVelocityY(left * magnitude)
                        .withVelocityX(forward * magnitude)
                        .withRotationalRate(rotation * SLOW_MAX_ROTATION_SPEED));
            }
        });
    }

    public Command slowestSwerveDrive(CommandXboxController joystick) {
        return run(() -> {
            double forward = -joystick.getLeftY();
            double left = -joystick.getLeftX();
            double rotation = -joystick.getRightX();
            double magnitude = Math.hypot(forward, left) * (SLOWEST_MAX_SPEED);
            magnitude = m_slewLimit.calculate(magnitude);
            checkEngageBrake(forward, left, rotation);

            if (m_brakeEngaged) {
                setControl(SwerveRequestStash.brake);
            } else {
                setControl(SwerveRequestStash.drive
                        .withVelocityY(left * magnitude)
                        .withVelocityX(forward * magnitude)
                        .withRotationalRate(rotation * SLOWEST_MAX_ROTATION_SPEED));
            }
        });
    }

    public Command swerveDefaultCommand(CommandXboxController joystick) {
        return run(() -> {
            double forward = -joystick.getLeftY();
            double left = -joystick.getLeftX();
            double rotation = -joystick.getRightX();
            double magnitude = Math.hypot(forward, left) * MAX_SPEED;
            magnitude = m_slewLimit.calculate(magnitude);
            checkEngageBrake(forward, left, rotation);
            joystickVx = forward * magnitude;
            joystickVy = left * magnitude;
            if (m_brakeEngaged) {
                setControl(SwerveRequestStash.brake);
            } else {
                setControl(SwerveRequestStash.drive
                        .withVelocityY(joystickVy)
                        .withVelocityX(joystickVx)
                        .withRotationalRate(rotation * MAX_ROTATION_SPEED));
            }
        });
    }

    public Command noDrive() {
        return run(() -> {
            setControl(SwerveRequestStash.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        });
    }

    public Command slowLoopRotate() {
            double initial_yaw = getYaw360();
            return run(() -> {
                setControl(SwerveRequestStash.drive.withVelocityX(0).withVelocityY(0).withRotationalRate((Math.PI / 180) * (5 * AutonConstants.LOOP_DEGREES_ROTATED/AutonConstants.SHOOTING_TIME))); // 1.5 * Rotation/Time : for loop path
            }).until(() -> (MathUtil.isNear(initial_yaw - LOOP_DEGREES_ROTATED, getYaw360(), 5) || MathUtil.isNear(initial_yaw + LOOP_DEGREES_ROTATED, getYaw360(), 5))).andThen(noDrive());
        }

    public Command unbeach() {
        return run(() -> {
            if ((getCurrentPose().getX() < ALLIANCE_ZONE_RED && getCurrentPose().getX() > MID_FIELD_X)
                    || (getCurrentPose().getX() < ALLIANCE_ZONE_BLUE)) {
                setControl(SwerveRequestStash.drive.withVelocityX(-3).withVelocityY(0).withRotationalRate(0));
            } else {
                setControl(SwerveRequestStash.drive.withVelocityX(3).withVelocityY(0).withRotationalRate(0));
            }
        }).until(() -> isUpright());
    }

    @Override
    public void periodic() {
        super.periodic();
        if (RED_ALLIANCE.isPresent()) {
            shooterArbiter.setCondition(shooterConditions.IN_ALLIANCE_ZONE, !inNeutralOrOpposingZone());
            shooterArbiter.setCondition(shooterConditions.SWERVE_SPEED_CORRECT,
                    isNotMovingTooFastOrTurning());
            hubPose = getHubPose();
            m_turretTargetAngle = calculateFieldRelativeTargetAngle();
        }
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

        public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    }

}

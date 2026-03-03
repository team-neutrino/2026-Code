package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.util.Subsystems.swerve;
import frc.robot.util.Constants.GlobalConstants;
import frc.robot.util.HubActiveStatus;
import frc.robot.util.MatchState;

public class DriverDashboard extends HubActiveStatus {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic matchTime = nt.getDoubleTopic("/DriverDashboard/Match Time");
    DoubleTopic remaningShiftTime = nt.getDoubleTopic("/DriverDashboard/Remaining Shift Time");
    BooleanTopic shiftActive = nt.getBooleanTopic("/DriverDashboard/Shift Active");
    StringTopic gameState = nt.getStringTopic("/DriverDashboard/Game State");
    StringTopic shiftNumber = nt.getStringTopic("/DriverDashboard/Shift Number");
    StringTopic shiftTimeColor = nt.getStringTopic("/DriverDashboard/Shift Time Color");

    BooleanEntry autonWon = nt.getBooleanTopic("/DriverDashboard/Auton Won").getEntry(false);

    final DoublePublisher matchTimePub;
    final DoublePublisher remainingShiftTimePub;
    final BooleanPublisher shiftActivePub;
    final StringPublisher gameStatePub;
    final StringPublisher shiftNumberPub;
    final StringPublisher shiftTimeColorPub;

    private final Field2d field = new Field2d();

    MatchState matchState = new MatchState();

    public DriverDashboard() {
        matchTimePub = matchTime.publish();
        matchTimePub.setDefault(0.0);

        remainingShiftTimePub = remaningShiftTime.publish();
        remainingShiftTimePub.setDefault(0.0);

        shiftActivePub = shiftActive.publish();
        shiftActivePub.setDefault(false);

        gameStatePub = gameState.publish();
        gameStatePub.setDefault("UNKNOWN");

        shiftNumberPub = shiftNumber.publish();
        shiftNumberPub.setDefault("NONE");

        shiftTimeColorPub = shiftTimeColor.publish();
        shiftTimeColorPub.setDefault("white");

        autonWon.setDefault(false);

        SmartDashboard.putData("Field", field);
    }

    public void periodic() {
        matchState.setAutoWinner(autonWon.get());
        matchTimePub.set(matchState.getMatchTime());

        double remaining = matchState.getRemainingShiftTime();
        remainingShiftTimePub.set(remaining);

        if (remaining <= 10) {
            shiftTimeColorPub.set("red");
        } else if (remaining >= 25) {
            shiftTimeColorPub.set("green");
        } else {
            shiftTimeColorPub.set("white");
        }

        shiftActivePub.set(matchState.isHubActive(GlobalConstants.RED_ALLIANCE.get() ? Alliance.RED : Alliance.BLUE));
        gameStatePub.set(matchState.getGameState());
        shiftNumberPub.set(matchState.getCurrentShiftName());
        field.setRobotPose(swerve.getCurrentPose());
    }
}
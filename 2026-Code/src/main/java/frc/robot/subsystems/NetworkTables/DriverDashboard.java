package frc.robot.subsystems.NetworkTables;

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
    DoubleTopic remainingShiftTime = nt.getDoubleTopic("/DriverDashboard/Remaining Shift Time");
    BooleanTopic shiftActive = nt.getBooleanTopic("/DriverDashboard/Shift Active");
    StringTopic gameState = nt.getStringTopic("/DriverDashboard/Game State");
    StringTopic shiftNumber = nt.getStringTopic("/DriverDashboard/Shift Number");
    BooleanTopic autonWon = nt.getBooleanTopic("/DriverDashboard/Auton Won");

    final DoublePublisher matchTimePub;
    final DoublePublisher remainingShiftTimePub;
    final BooleanPublisher shiftActivePub;
    final StringPublisher gameStatePub;
    final StringPublisher shiftNumberPub;
    final BooleanPublisher autonWonPub;

    private boolean wonAuton;
    private final Field2d field = new Field2d();
    MatchState matchState = new MatchState();

    public DriverDashboard() {
        matchTimePub = matchTime.publish();
        matchTimePub.setDefault(0.0);

        remainingShiftTimePub = remainingShiftTime.publish();
        remainingShiftTimePub.setDefault(0.0);

        shiftActivePub = shiftActive.publish();
        shiftActivePub.setDefault(false);

        gameStatePub = gameState.publish();
        gameStatePub.setDefault("UNKNOWN");

        shiftNumberPub = shiftNumber.publish();
        shiftNumberPub.setDefault("NONE");

        autonWonPub = autonWon.publish();
        autonWonPub.setDefault(false);

        SmartDashboard.putData("Field", field);
    }

    public void periodic() {
        update();
        System.out.println(Math.round(matchState.getMatchTime()));
        if (hasValidGameData()) {
            wonAuton = GlobalConstants.RED_ALLIANCE.get()
                    ? isRedHubActive()
                    : isBlueHubActive();
            autonWonPub.set(wonAuton);
        }

        matchState.setAutoWinner(wonAuton);
        matchTimePub.set(matchState.getMatchTime());

        double remaining = matchState.getRemainingShiftTime();
        remainingShiftTimePub.set(remaining);

        shiftActivePub.set(matchState.isHubActive(GlobalConstants.RED_ALLIANCE.get() ? Alliance.RED : Alliance.BLUE));
        gameStatePub.set(matchState.getGameState());
        shiftNumberPub.set(matchState.getCurrentShiftName());
        field.setRobotPose(swerve.getCurrentPose());
    }
}
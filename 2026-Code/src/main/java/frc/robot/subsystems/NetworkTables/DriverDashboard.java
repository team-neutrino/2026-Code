package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import frc.robot.util.Constants.GlobalConstants;
import frc.robot.util.HubActiveStatus;
import frc.robot.util.MatchState;

public class DriverDashboard extends HubActiveStatus {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic matchTime = nt.getDoubleTopic("/DriverDashboard/match_time");
    DoubleTopic remaningShiftTime = nt.getDoubleTopic("/DriverDashboard/remaning_shift_time");
    BooleanTopic shiftActive = nt.getBooleanTopic("/DriverDashboard/shift_active");
    StringTopic gameState = nt.getStringTopic("/DriverDashboard/climb_game_state");
    BooleanTopic activeFirst = nt.getBooleanTopic("/DriverDashboard/active_first");
    final DoublePublisher matchTimePub;
    final DoublePublisher remainingShiftTimePub;
    final BooleanPublisher shiftActivePub;
    final StringPublisher gameStatePub;
    final BooleanPublisher activeFirstPub;
    boolean weWon = false;
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

        activeFirstPub = activeFirst.publish();
        activeFirstPub.setDefault(false);
    }

    public void periodic() {
        matchState.setAutoWinner(weWon);
        matchTimePub.set(matchState.getMatchTime());
        remainingShiftTimePub.set(matchState.getRemainingShiftTime());
        shiftActivePub.set(matchState.isHubActive(GlobalConstants.RED_ALLIANCE.get() ? Alliance.RED : Alliance.BLUE));
        gameStatePub.set(matchState.getGameState());
        // activeFirstPub.set(matchState.);
    }
}
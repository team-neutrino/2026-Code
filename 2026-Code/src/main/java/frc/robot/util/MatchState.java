package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class MatchState extends HubActiveStatus {
    private boolean allianceWonAuto = false;

    public void setAutoWinner(boolean weWon) {
        allianceWonAuto = weWon;
    }

    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    public String getGameState() {
        double time = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous())
            return "AUTO";
        if (time > 130)
            return "TRANSITION";
        if (time <= 30)
            return "ENDGAME";
        return "TELEOP";
    }

    public double getRemainingShiftTime() {
        double time = DriverStation.getMatchTime();
        String state = getGameState();

        switch (state) {
            case "AUTO":
                return time;
            case "TRANSITION":
                return time - 130;
            case "ENDGAME":
                return time;
            case "TELEOP":
                double teleopElapsed = 130 - time;
                return 25 - (teleopElapsed % 25);
            default:
                return 0;
        }
    }

    public String getCurrentShiftName() {
        if (!getGameState().equals("TELEOP"))
            return "NONE";

        double matchTime = DriverStation.getMatchTime();

        if (matchTime > 130)
            return "Transition";
        if (matchTime > 105)
            return "Shift 1";
        if (matchTime > 80)
            return "Shift 2";
        if (matchTime > 55)
            return "Shift 3";
        if (matchTime > 30)
            return "Shift 4";
        return "End Game";
    }
}
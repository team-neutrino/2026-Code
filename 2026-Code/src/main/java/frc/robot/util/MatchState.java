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

    public int getCurrentShiftNumber() {
        if (!getGameState().equals("TELEOP"))
            return 0;

        double teleopElapsed = 130 - DriverStation.getMatchTime();
        return (int) (teleopElapsed / 25) + 1;
    }
}
package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class HubActiveStatus {
    public enum Alliance {
        RED,
        BLUE
    }

    private Alliance inactiveFirst = null;

    public void update() {
        if (inactiveFirst != null) {
            return;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'R':
                    inactiveFirst = Alliance.RED;
                    break;
                case 'B':
                    inactiveFirst = Alliance.BLUE;
                    break;
                default:
                    System.out.println("Warning: no alliance selected to go inactive first");
                    break;
            }
        }
    }

    private boolean hasValidGameData() {
        return inactiveFirst != null;
    }

    private boolean isHubActive(Alliance alliance) {
        if (!hasValidGameData()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();

        // auto/transition + endgame
        if (matchTime <= 30.0 || matchTime >= 130) {
            return true;
        }

        double shiftTime = 130.0 - matchTime;

        int shiftIndex = (int) (shiftTime / 25.0); // 0–3

        boolean inactiveThisShift;

        if (shiftIndex % 2 == 0) {
            inactiveThisShift = (alliance == inactiveFirst);
        } else {
            inactiveThisShift = (alliance != inactiveFirst);
        }

        return !inactiveThisShift;
    }

    public boolean isRedHubActive() {
        return isHubActive(Alliance.RED);
    }

    public boolean isBlueHubActive() {
        return isHubActive(Alliance.BLUE);
    }

}

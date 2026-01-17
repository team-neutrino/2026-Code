package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;

public class LED extends SubsystemBase {

    private String gameData;
    gameData = DriverStation.getGameSpecificmMessage();

    public LED() {}

    //LED Color indicates with alliance is activated
    public void setAllianceColor() {
        if(gameData.length() > 0) {
        switch (gameData.charAt(0)) {
            case 'B':
                //blue color turn on
                break;
            case 'R':
                //red color turn on
                break;
            default:
                //turn off
                break;
        }
    } else{
        //turn off
    }
    }

    public void periodic() {
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.StringPublisher;
<<<<<<< HEAD
import frc.robot.util.HubActiveStatus;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {
=======

public class LED extends SubsystemBase {

    private String gameData;
>>>>>>> origin/LEDs
    private double gameTime;

    private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
    private StringTopic m_color_topic = m_nt.getStringTopic("/LED/color");
    private StringTopic m_state_topic = m_nt.getStringTopic("/LED/state");

    private final StringPublisher m_color_pub;
    private final StringPublisher m_state_pub;

<<<<<<< HEAD
    private HubActiveStatus m_hub_status = Subsystem.hubState;

=======
>>>>>>> origin/LEDs
    public LED() {
        m_color_pub = m_color_topic.publish();
        m_state_pub = m_state_topic.publish();
    }

<<<<<<< HEAD
    @Override
    public void periodic() {
        gameTime = DriverStation.getMatchTime();

        // blink 5 seconds before alliance shift changes
        if (gameTime <= 135 && gameTime >= 130) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (gameTime <= 110 && gameTime >= 105) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (gameTime <= 85 && gameTime >= 80) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (gameTime <= 60 && gameTime >= 55) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (gameTime <= 35 && gameTime >= 30) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        // alignment to shoot/pass/climb. determine based on drive to point
        //
        // moving to point - pink
        // m_color_pub.set("pink");
        // m_state_pub.set("solid");
        //
        // at point - green
        // m_color_pub.set("green");
        // m_state_pub.set("solid");

        // hopper full. determine based on sensor in hopper
        //
        // when beambreak is broken - orange
        // m_color_pub.set("orange");
        // m_state_pub.set("solid");

        // hopper empty ? potentially deteremine based on a robot sensor

        // red hub active
        if (m_hub_status.isRedHubActive() && !m_hub_status.isBlueHubActive()) {
            m_color_pub.set("red");
            m_state_pub.set("solid");
        }

        // blue hub active
        else if (m_hub_status.isBlueHubActive() && !m_hub_status.isRedHubActive()) {
            m_color_pub.set("blue");
            m_state_pub.set("solid");
        }

        // default to purple (during auton, transition shift, endgame)
        m_color_pub.set("purple");
        m_state_pub.set("solid");

    }

=======
    // LED Color indicates with alliance is activated
    public void setAllianceColor() {
        gameData = DriverStation.getGameSpecificMessage(); // update gameData
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    // blue color turn on
                    break;
                case 'R':
                    // red color turn on
                    break;
                default:
                    // turn off
                    break;
            }
        } else {
            // turn off
        }
    }

    public void setTransition() {
        gameTime = DriverStation.getMatchTime();
        if (gameTime < 0) {
            // math
        }

    }

    public void periodic() {
        setAllianceColor();
        setTransition();
    }
>>>>>>> origin/LEDs
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.StringPublisher;

import frc.robot.util.HubActiveStatus;
import frc.robot.util.Subsystems2026;

public class LED extends SubsystemBase {
    private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
    private StringTopic m_color_topic = m_nt.getStringTopic("/LED/color");
    private StringTopic m_state_topic = m_nt.getStringTopic("/LED/state");

    private final StringPublisher m_color_pub;
    private final StringPublisher m_state_pub;

    private double m_gameTime;
    private HubActiveStatus m_hub_status = Subsystems2026.hubState;
    private Index m_index = Subsystems2026.index;

    public LED() {
        m_color_pub = m_color_topic.publish();
        m_state_pub = m_state_topic.publish();
    }

    @Override
    public void periodic() {
        m_gameTime = DriverStation.getMatchTime();

        // blink 5 seconds before alliance shift changes
        if (m_gameTime <= 135 && m_gameTime >= 130) { // 2:15-2:10
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (m_gameTime <= 110 && m_gameTime >= 105) { // 1:50-1:45
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (m_gameTime <= 85 && m_gameTime >= 80) { // 1:25-1:20
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (m_gameTime <= 60 && m_gameTime >= 55) { // 1:00-0:55
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (m_gameTime <= 35 && m_gameTime >= 30) {
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

        // when hopper full - orange
        if (m_index.fullCapacity()) {
            m_color_pub.set("orange");
            m_state_pub.set("solid");
        }

        // hopper empty ? potentially deteremine based on a robot sensor

        // red hub active
        if (m_hub_status.isRedHubActive() && !m_hub_status.isBlueHubActive()) {
            m_color_pub.set("red");
            m_state_pub.set("solid");
            return;
        }

        // blue hub active
        else if (m_hub_status.isBlueHubActive() && !m_hub_status.isRedHubActive()) {
            m_color_pub.set("blue");
            m_state_pub.set("solid");
            return;
        }

        // default to purple (during auton, transition shift, endgame)
        m_color_pub.set("purple");
        m_state_pub.set("solid");

        // System.out.println(DriverStation.getGameSpecificMessage());
        // System.out.println("blue hub: " + m_hub_status.isBlueHubActive());
        // System.out.println("red hub: " + m_hub_status.isRedHubActive());

    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.util.HubActiveStatus;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {
    private double m_gameTime;

    private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
    private StringTopic m_color_topic = m_nt.getStringTopic("/LED/color");
    private StringTopic m_state_topic = m_nt.getStringTopic("/LED/state");

    private final StringPublisher m_color_pub;
    private final StringPublisher m_state_pub;

    private HubActiveStatus m_hub_status = Subsystem.hubState;

    private boolean m_hopperBeam = false;
    private DigitalInput m_hopperBreambreak = new DigitalInput(5); // random number

    public LED() {
        m_color_pub = m_color_topic.publish();
        m_state_pub = m_state_topic.publish();
    }

    @Override
    public void periodic() {
        m_gameTime = DriverStation.getMatchTime();
        m_hopperBeam = m_hopperBreambreak.get(); // from 2024 intake code, don't really know what it's for?

        // blink 5 seconds before alliance shift changes
        if (m_gameTime <= 135 && m_gameTime >= 130) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (m_gameTime <= 110 && m_gameTime >= 105) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (m_gameTime <= 85 && m_gameTime >= 80) {
            m_color_pub.set("white");
            m_state_pub.set("blink");
            return;
        }

        else if (m_gameTime <= 60 && m_gameTime >= 55) {
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

        // hopper full. determine based on sensor in hopper

        // when beambreak is broken - orange
        if (m_hopperBeam) {
            m_color_pub.set("orange");
            m_state_pub.set("solid");
        }

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
}

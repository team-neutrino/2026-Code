package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.util.Constants;
import frc.robot.util.Subsystems;
import frc.robot.util.HubActiveStatus;

import java.util.GregorianCalendar;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.LEDConstants.CANDLE_ID, "rio");

    private double m_gameTime;
    private HubActiveStatus m_hub_status = Subsystems.hubState;
    private Index m_index = Subsystems.index;

    private int[] white = { 64, 64, 64 }; // blink countdown
    private int[] red = { 64, 0, 0 }; // red hub
    private int[] orange = { 84, 18, 0 }; // full hopper
    private int[] yellow = { 64, 64, 0 };
    private int[] green = { 0, 64, 0 }; // empty hopper
    private int[] blue = { 0, 0, 64 }; // blue hub
    private int[] purple = { 64, 0, 64 }; // default (when both hubs active: auton, transition shift, endgame)
    private int[] black = { 0, 0, 0 }; // off

    public LED() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.stripType = LEDStripType.RGB;
        m_candle.configAllSettings(configAll);
    }

    @Override
    public void periodic() {
        m_gameTime = DriverStation.getMatchTime();

        // blink 5 seconds before alliance shift changes
        if (m_gameTime <= 135 && m_gameTime >= 130) { // 2:15-2:10
            m_candle.setLEDs(white);
            return;
        }

        else if (m_gameTime <= 110 && m_gameTime >= 105) { // 1:50-1:45
            m_candle.setLEDs(white);
            return;
        }

        else if (m_gameTime <= 85 && m_gameTime >= 80) { // 1:25-1:20
            m_candle.setLEDs(white);
            return;
        }

        else if (m_gameTime <= 60 && m_gameTime >= 55) { // 1:00-0:55
            m_candle.setLEDs(white);
            return;
        }

        else if (m_gameTime <= 35 && m_gameTime >= 30) {
            m_candle.setLEDs(white);
            return;
        }

        // alignment to shoot/pass/climb. determine based on drive to point
        //
        // moving to point = pink
        // m_color_pub.set("pink");
        // m_state_pub.set("solid");
        //
        // at point = green
        // m_color_pub.set("green");
        // m_state_pub.set("solid");

        // when hopper full = orange
        if (m_index.fullCapacity()) {
            m_candle.setLEDs(orange);
            return;
        }

        // hopper empty = green (color could change, sarah randomly picked)
        if (m_index.isHopperEmpty()) {
            m_candle.setLEDs(green);
            return;
        }

        // red hub active
        if (m_hub_status.isRedHubActive() && !m_hub_status.isBlueHubActive()) {
            m_candle.setLEDs(red);
            return;
        }

        // blue hub active
        else if (m_hub_status.isBlueHubActive() && !m_hub_status.isRedHubActive()) {
            m_candle.setLEDs(blue);
            return;
        }

        // default to purple (during auton, transition shift, endgame)
        m_candle.setLEDs(purple);
    }
}

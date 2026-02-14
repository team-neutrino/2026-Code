package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Constants;
import frc.robot.util.Subsystems;
import frc.robot.util.HubActiveStatus;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.*;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.LEDConstants.CANDLE_ID, "rio");

    private double m_gameTime;
    private HubActiveStatus m_hub_status = Subsystems.hubState;
    private Index m_index = Subsystems.index;

    private RGBWColor white = new RGBWColor(64, 64, 64); // blink countdown
    private RGBWColor red = new RGBWColor(64, 0, 0); // red hub
    private RGBWColor orange = new RGBWColor(84, 18, 0); // full hopper
    private RGBWColor yellow = new RGBWColor(64, 64, 0);
    private RGBWColor green = new RGBWColor(0, 64, 0); // empty hopper
    private RGBWColor blue = new RGBWColor(0, 0, 64); // blue hub
    private RGBWColor purple = new RGBWColor(64, 0, 64); // default (when both hubs active: auton, transition, endgame)
    private RGBWColor black = new RGBWColor(0, 0, 0); // off

    public LED() {
        CANdleConfiguration configAll = new CANdleConfiguration();

        m_candle.getConfigurator().apply(configAll);
    }

    @Override
    public void periodic() {
        m_gameTime = DriverStation.getMatchTime();
        m_candle.setControl(new SolidColor(8, 20).withColor(purple));
        // blink 5 seconds before alliance shift changes
        if (m_gameTime <= 135 && m_gameTime >= 130) { // 2:15-2:10
            return;
        }

        else if (m_gameTime <= 110 && m_gameTime >= 105) { // 1:50-1:45
            return;
        }

        else if (m_gameTime <= 85 && m_gameTime >= 80) { // 1:25-1:20
            return;
        }

        else if (m_gameTime <= 60 && m_gameTime >= 55) { // 1:00-0:55
            return;
        }

        else if (m_gameTime <= 35 && m_gameTime >= 30) {
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
            m_candle.setControl(new SolidColor(8, 20).withColor(orange));
            return;
        }

        // hopper empty = green (color could change, sarah randomly picked)
        if (m_index.isHopperEmpty()) {
            m_candle.setControl(new SolidColor(8, 20).withColor(green));
            return;
        }

        // red hub active
        if (m_hub_status.isRedHubActive() && !m_hub_status.isBlueHubActive()) {
            m_candle.setControl(new SolidColor(8, 20).withColor(red));
            return;
        }

        // blue hub active
        else if (m_hub_status.isBlueHubActive() && !m_hub_status.isRedHubActive()) {
            m_candle.setControl(new SolidColor(8, 20).withColor(blue));
            return;
        }

        // default to purple (during auton, transition shift, endgame)
    }
}

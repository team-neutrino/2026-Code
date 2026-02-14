package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.util.Constants.LEDConstants.*;
import frc.robot.util.Subsystems;
import frc.robot.util.HubActiveStatus;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.*;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CANDLE_ID, "rio");

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

        // blink 5 seconds before all alliance shift changes
        // if ((m_gameTime <= 135 && m_gameTime >= 130) || (m_gameTime <= 110 && m_gameTime >= 105)
        //        || (m_gameTime <= 85 && m_gameTime >= 80) || (m_gameTime <= 60 && m_gameTime >= 55)
        //        || (m_gameTime <= 35 && m_gameTime >= 30)) { // 2:15-2:10, 1:50-1:45, 1:25-1:20, 1:00-0:55
        //    m_candle.setControl(new StrobeAnimation(START_INDEX, END_INDEX).withColor(white));
        //    System.out.println("white");
        //    return;
        //}

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
        // if (m_index.fullCapacity()) {
        // m_candle.setControl(new SolidColor(START_INDEX,
        // END_INDEX).withColor(orange));
        // return;
        // }

        // hopper empty = green (color could change, sarah randomly picked)
        // if (m_index.isHopperEmpty()) {
        // m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(green));
        // return;
        // }

        // red hub active
        // else if (m_hub_status.isRedHubActive() && !m_hub_status.isBlueHubActive()) {
        // m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(red));
        // return;
        // }

        // blue hub active
        // else if (m_hub_status.isBlueHubActive() && !m_hub_status.isRedHubActive()) {
        // m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(blue));
        // return;
        // }

        // else { // default to purple (during auton, transition shift, endgame)
        // m_candle.setControl(new SolidColor(START_INDEX,
        // END_INDEX).withColor(purple));
        // }

        m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(purple));
        System.out.println("LEDing");
    }
}

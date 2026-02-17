package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;
import static edu.wpi.first.wpilibj.RobotController.*;

import static frc.robot.util.Constants.LEDConstants.*;
import frc.robot.util.Subsystems;
import frc.robot.util.HubActiveStatus;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.math.filter.Debouncer;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CANDLE_ID, "rio");

    private boolean m_isDisabled;
    private double m_gameTime;
    private HubActiveStatus m_hub_status = Subsystems.hubState;
    private Index m_index = Subsystems.index;

    private Debouncer m_debouncer = new Debouncer(VOLTAGE_WARNING_DEBOUNCED_TIME);

    private RGBWColor white = new RGBWColor(64, 64, 64); // blink before shift changes
    private RGBWColor red = new RGBWColor(64, 0, 0); // red hub
    private RGBWColor orange = new RGBWColor(84, 18, 0);
    private RGBWColor yellow = new RGBWColor(64, 64, 0);
    private RGBWColor green = new RGBWColor(0, 64, 0); // full hopper
    private RGBWColor blue = new RGBWColor(0, 0, 64); // blue hub
    private RGBWColor purple = new RGBWColor(64, 0, 64); // default (when both hubs active: auton, transition, endgame)

    public LED() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        m_candle.getConfigurator().apply(configAll);
    }

    public boolean under12V() {
        return getBatteryVoltage() < 12;
    }

    public boolean under12For1() {
        return m_debouncer.calculate(under12V());
    }

    // public void batteryStuff() {
    // // under 10 during a match
    // if (DriverStation.isFMSAttached() || ) {
    // m_candle.setControl(new SingleFadeAnimation(7,
    // 7).withColor(orange).withSlot(0));
    // }
    // }

    @Override
    public void periodic() {
        m_isDisabled = DriverStation.isDisabled();
        m_gameTime = DriverStation.getMatchTime();

        // blink 5 seconds before all alliance shift changes
        // 2:15-2:10, 1:50-1:45, 1:25-1:20, 1:00-0:55, 0:35-0:30
        if ((m_gameTime <= 135 && m_gameTime >= 130) || (m_gameTime <= 110 &&
                m_gameTime >= 105)
                || (m_gameTime <= 85 && m_gameTime >= 80) || (m_gameTime <= 60 && m_gameTime >= 55)
                || (m_gameTime <= 35 && m_gameTime >= 30)) {
            m_candle.setControl(new StrobeAnimation(START_INDEX, END_INDEX).withColor(white).withSlot(0));
        }

        // battery voltage under 12 for more than a minute (not during a match)
        else if (under12For1() && !DriverStation.isFMSAttached()) {
            m_candle.setControl(new EmptyAnimation(0));
            m_candle.setControl(new SingleFadeAnimation(START_INDEX, END_INDEX).withColor(orange).withSlot(0));
            System.out.println("Battery under 12V for 1 minute... Change the battery!!!");
        }

        // when hopper full = green
        else if (m_index.fullCapacity()) {
            m_candle.setControl(new EmptyAnimation(0));
            m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(green));
        }

        // red hub active
        else if (m_hub_status.isRedHubActive() && !m_hub_status.isBlueHubActive()) {
            m_candle.setControl(new EmptyAnimation(0));
            m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(red));
        }

        // blue hub active
        else if (m_hub_status.isBlueHubActive() && !m_hub_status.isRedHubActive()) {
            m_candle.setControl(new EmptyAnimation(0));
            m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(blue));
        }

        // rainbow if disabled and not connected to FMS
        else if (m_isDisabled && !DriverStation.isFMSAttached()) {
            m_candle.setControl(new EmptyAnimation(0));
            m_candle.setControl(new RainbowAnimation(START_INDEX, END_INDEX).withSlot(0).withBrightness(0.1));
        }

        // default to purple (auton, transition shift, endgame)
        else {
            m_candle.setControl(new EmptyAnimation(0));
            m_candle.setControl(new SolidColor(START_INDEX, END_INDEX).withColor(purple));
        }

        System.out.println("LEDing");
    }
}

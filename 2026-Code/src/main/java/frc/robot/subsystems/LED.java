package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.RobotController.*;

import static frc.robot.util.Constants.LEDConstants.*;

import java.util.HashMap;

import static frc.robot.util.Subsystems.*;
import frc.robot.util.HubActiveStatus;
import frc.robot.util.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.math.filter.Debouncer;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CANDLE_ID, "rio");

    private boolean m_isDisabled;
    private double m_gameTime;

    private Debouncer m_debouncer = new Debouncer(VOLTAGE_WARNING_DEBOUNCED_TIME);

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
            m_candle.setControl(new StrobeAnimation(START_INDEX, END_INDEX).withColor(WHITE).withSlot(0));
            return;
        }

        // battery voltage under 12 for more than a minute (not during a match)
        else if (under12For1() && !DriverStation.isFMSAttached()) {
            m_candle.setControl(new SingleFadeAnimation(START_INDEX,
                    MID_INDEX).withColor(ORANGE).withSlot(0));
            System.out.println("Battery under 12V for 1 minute... Change the battery!!!");
        }

        ShooterConditionsCheck();
        OdometryCheck();
    }

    private void ShooterConditionsCheck() {
        if (!shooterArbiter.readyToFire()) {
            HashMap<ShooterConstants.shooterConditions, Boolean> conditions = shooterArbiter.getConditions();
            ShooterConstants.shooterConditions[] shooterValues = ShooterConstants.shooterConditions.values();
            for (int i = 0; i < shooterValues.length; i++) {
                if (!conditions.get(shooterValues[i])) {
                    m_candle.setControl(
                            new SolidColor(START_INDEX, MID_INDEX).withColor(COLOR_MAP.get(shooterValues[i])));
                    break;
                }
            }
        } else {
            m_candle.setControl(new SolidColor(START_INDEX, MID_INDEX).withColor(GREEN));
        }
    }

    private void OdometryCheck() {
        if (!vision.hasTag()) {
            m_candle.setControl(new SolidColor(MID_INDEX + 1, END_INDEX).withColor(ORANGE));
        } else {
            m_candle.setControl(new SolidColor(MID_INDEX + 1, END_INDEX).withColor(GREEN));
        }
    }
}
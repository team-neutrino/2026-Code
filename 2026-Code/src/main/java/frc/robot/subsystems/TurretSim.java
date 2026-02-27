package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.robot.util.Constants.TurretConstants.*;

/**
 * Simulation class for the Turret subsystem.
 * Simulates a rotating turret mechanism with no gravity effects.
 */
public class TurretSim {
    private final TalonFXSimState m_motorSim;
    private final CANcoderSimState m_encoderSim;

    // Simple rotational inertia simulation (no gravity)
    private final SingleJointedArmSim m_turretSim;

    // Damping coefficient to reduce oscillation (simulates friction)
    private static final double DAMPING_COEFFICIENT = 0.50; // Nm/(rad/s) - adjust to tune behavior

    // Visualization
    private final Mechanism2d m_mech2d;
    private final MechanismRoot2d m_turretRoot;
    private final MechanismLigament2d m_turretLigament;

    // Gear ratios
    private final double m_rotorToSensorRatio;
    private final double m_sensorToMechanismRatio;

    /**
     * Creates a new TurretSim.
     * 
     * @param motor   The turret motor
     * @param encoder The turret encoder
     */
    public TurretSim(TalonFX motor, CANcoder encoder) {
        m_motorSim = motor.getSimState();
        m_encoderSim = encoder.getSimState();

        m_rotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;
        m_sensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;

        // Calculate total gear ratio from motor to mechanism
        double totalGearRatio = m_rotorToSensorRatio * m_sensorToMechanismRatio;

        // Create arm simulation with no gravity effect (0 degrees minimum angle,
        // infinite max)
        // Using a Kraken X44 motor with the gear ratio
        // Estimating moment of inertia for a turret (can be tuned)
        double armLengthMeters = 0.2; // Approximate radius for visualization (30 cm)
        double massKg = 2.0; // Approximate turret mass (can be tuned)

        m_turretSim = new SingleJointedArmSim(
                DCMotor.getKrakenX44(1), // 1 Kraken X44 motor
                totalGearRatio,
                SingleJointedArmSim.estimateMOI(armLengthMeters, massKg), // Moment of inertia
                armLengthMeters,
                -2 * Math.PI, // Min angle (allow full rotation)
                2 * Math.PI, // Max angle (allow full rotation)
                false, // Simulate gravity? NO - turret is horizontal
                0.0 // Starting angle
        );

        // Set simulation to use Chassis frame (fixed reference)
        m_motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_encoderSim.Orientation = ChassisReference.CounterClockwise_Positive;

        // Create visualization
        m_mech2d = new Mechanism2d(3, 3);
        m_turretRoot = m_mech2d.getRoot("Turret", 1.5, 1.5);
        m_turretLigament = m_turretRoot.append(
                new MechanismLigament2d("TurretArm", 0.5, 0, 6, new Color8Bit(Color.kOrange)));

        SmartDashboard.putData("Turret Sim", m_mech2d);
    }

    /**
     * Updates the simulation. Should be called periodically in
     * simulationPeriodic().
     */
    public void updateSimulation() {
        // Set the input voltage from what the motor is trying to output
        double motorVoltage = m_motorSim.getMotorVoltage();

        // Apply damping to reduce oscillation (simulates friction and mechanical
        // damping)
        // Damping torque opposes velocity: T_damping = -b * omega
        double currentVelocity = m_turretSim.getVelocityRadPerSec();
        double dampingTorque = -DAMPING_COEFFICIENT * currentVelocity;

        // Convert damping torque to equivalent voltage reduction at the motor
        // This is a simplified model: in reality, friction is more complex
        DCMotor motor = DCMotor.getKrakenX44(1);
        double totalGearRatio = m_rotorToSensorRatio * m_sensorToMechanismRatio;
        double dampingVoltage = (dampingTorque / totalGearRatio) / motor.KtNMPerAmp * motor.rOhms;

        m_turretSim.setInputVoltage(motorVoltage + dampingVoltage);

        // Update the simulation with the timestep
        m_turretSim.update(0.020); // 20ms loop time

        // Get the simulated angle and velocity
        double angleRadians = m_turretSim.getAngleRads();
        double velocityRadPerSec = m_turretSim.getVelocityRadPerSec();

        // Convert mechanism angle (radians) to mechanism rotations
        double mechanismRotations = angleRadians / (2.0 * Math.PI);
        double mechanismRotationsPerSec = velocityRadPerSec / (2.0 * Math.PI);

        // Convert to CANcoder rotations (CANcoder is on the sensor side, after
        // rotor-to-sensor but before sensor-to-mechanism)
        // mechanismRotations * sensorToMechanismRatio = sensorRotations
        double sensorPositionRotations = mechanismRotations * m_sensorToMechanismRatio;
        double sensorVelocityRPS = mechanismRotationsPerSec * m_sensorToMechanismRatio;

        // Set the simulated encoder position
        m_encoderSim.setRawPosition(sensorPositionRotations);
        m_encoderSim.setVelocity(sensorVelocityRPS);

        // The TalonFX rotor position: sensorRotations / rotorToSensorRatio =
        // rotorRotations
        double motorPositionRotations = sensorPositionRotations / m_rotorToSensorRatio;
        double motorVelocityRPS = sensorVelocityRPS / m_rotorToSensorRatio;

        m_motorSim.setRawRotorPosition(motorPositionRotations);
        m_motorSim.setRotorVelocity(motorVelocityRPS);

        // Update supply voltage
        m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Update visualization - convert angle to degrees for display
        double angleDegrees = Units.radiansToDegrees(angleRadians);
        m_turretLigament.setAngle(angleDegrees);

        // Update color based on whether at target (this would need to be set
        // externally)
        // For now, just show orange when moving, green when stopped
        if (Math.abs(velocityRadPerSec) < 0.1) {
            m_turretLigament.setColor(new Color8Bit(Color.kGreen));
        } else {
            m_turretLigament.setColor(new Color8Bit(Color.kOrange));
        }
    }

    /**
     * Gets the current simulated angle in degrees.
     * 
     * @return The turret angle in degrees
     */
    public double getSimulatedAngleDegrees() {
        return Units.radiansToDegrees(m_turretSim.getAngleRads());
    }

    /**
     * Gets the current simulated velocity in degrees per second.
     * 
     * @return The turret velocity in degrees per second
     */
    public double getSimulatedVelocityDegreesPerSec() {
        return Units.radiansToDegrees(m_turretSim.getVelocityRadPerSec());
    }

    /**
     * Resets the simulation to a specific angle.
     * 
     * @param angleDegrees The angle to reset to, in degrees
     */
    public void resetSimulation(double angleDegrees) {
        double angleRadians = Units.degreesToRadians(angleDegrees);
        m_turretSim.setState(angleRadians, 0.0);

        // Update hardware sims to match
        // Convert mechanism angle to rotations, then apply gear ratios
        double mechanismRotations = angleRadians / (2.0 * Math.PI);
        double sensorPositionRotations = mechanismRotations * m_sensorToMechanismRatio;
        m_encoderSim.setRawPosition(sensorPositionRotations);

        double motorPositionRotations = sensorPositionRotations / m_rotorToSensorRatio;
        m_motorSim.setRawRotorPosition(motorPositionRotations);
    }
}

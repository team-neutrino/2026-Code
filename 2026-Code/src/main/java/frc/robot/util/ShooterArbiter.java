package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import static frc.robot.util.Constants.ShooterConstants.*;

/**
 * The class that controls whether or not the shooter can fire.
 * <p>
 * To add a new condition to this class, go to the ShooterConstants file, and
 * add a new state to the shooterConditions enum.
 * <p>
 * Then, in the subsystem the condition pulls from, use the
 * {@link #setCondition(shooterConditions, boolean)} method to set the
 * condition.
 */

@SuppressWarnings("unused")
public class ShooterArbiter {
    private HashMap<shooterConditions, Boolean> m_conditions = new HashMap<shooterConditions, Boolean>();

    /**
     * Creates a new ShooterArbiter, which will check whether or not the shooter is
     * ready to fire.
     */
    ShooterArbiter() {
        for (shooterConditions condition : shooterConditions.values()) {
            m_conditions.put(condition, true); // if this value is true, it will be more lenient (unmodified values will
                                               // not prevent from shooting)
        }
    }

    /**
     * Check whether or not the shooter is ready to fire.
     * 
     * @return Whether or not the shooter is ready to fire. Will be false if any
     *         conditions are unmet.
     */
    public boolean readyToFire() {
        for (Map.Entry<shooterConditions, Boolean> val : m_conditions.entrySet()) {

        if (!val.getValue()) {
        System.out.println(val.getKey());
        }
        }
        // uncomment for debugging

        return !(m_conditions.containsValue(false));
    }

    /**
     * Set one of the conditions in the shooter arbiter to true or false. If any are
     * false, the shooter will not run.
     * 
     * @param condition The condition to set, from the shooterConditions enum in
     *                  ShooterConstants.
     * @param newValue  The boolean value to set it to.
     */
    public void setCondition(shooterConditions condition, boolean newValue) {
        m_conditions.put(condition, newValue);
    }

    /**
     * Gets a singular value from the shooter arbiter.
     * @param condition A value from the shooterConditions enum.
     * @return A boolean value from the shooter arbiter.
     */
    public boolean getCondition(shooterConditions condition) {
        return m_conditions.get(condition);
    }
}

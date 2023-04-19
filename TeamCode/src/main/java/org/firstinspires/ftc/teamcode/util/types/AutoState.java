package org.firstinspires.ftc.teamcode.util.types;

import org.firstinspires.ftc.teamcode.autonomous.Auto;

/**
 * A complete representation of an autonomous state
 */
public class AutoState {
    public final Color color;
    public final StartPosition position;
    public final long delay;
    public final AutoType autoType;

    /**
     * Create auto state object with specified parameters
     * @param color The color, red or blue, of the auto state
     * @param position The start position of the auto state
     * @param autoType The type of the auto state, long or sweat
     * @param delay The start delay
     */
    public AutoState(Color color, StartPosition position, AutoType autoType, long delay) {
        this.color = color;
        this.position = position;
        this.delay = delay;
        this.autoType = autoType;
    }
    
    /**
     * Constructor with no delay.
     */
    public AutoState(Color color, StartPosition position, AutoType autoType) {
        this(color, position, autoType, 0);
    }
    
    /**
     * Checks if this autostate is equal to another. Ignores delay.
     * @param autoState The autostate to compare to
     * @return Whether the two autostates are the same, ignoring delay
     */
    public boolean isEqualType(AutoState autoState) {
        return autoState.color == color && autoState.position == position && autoState.autoType == autoType;
    }
}

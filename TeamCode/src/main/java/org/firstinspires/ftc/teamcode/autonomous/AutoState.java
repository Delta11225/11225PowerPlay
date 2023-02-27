package org.firstinspires.ftc.teamcode.autonomous;

public class AutoState {
    public final Color color;
    public final StartPosition position;
    public final long delay;
    public final AutoType autoType;

    public AutoState(Color color, StartPosition position, AutoType autoType, long delay) {
        this.color = color;
        this.position = position;
        this.delay = delay;
        this.autoType = autoType;
    }
}

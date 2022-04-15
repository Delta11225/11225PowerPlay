package org.firstinspires.ftc.teamcode.competition.util;

import org.firstinspires.ftc.teamcode.competition.types.Color;
import org.firstinspires.ftc.teamcode.competition.types.ParkingMethod;
import org.firstinspires.ftc.teamcode.competition.types.StartPosition;

public class AutoState {
    public final Color color;
    public final StartPosition position;
    public final ParkingMethod parkMethod;
    public final long delay;

    public AutoState(Color color, StartPosition position, ParkingMethod parkMethod, long delay) {
        this.color = color;
        this.position = position;
        this.parkMethod = parkMethod;
        this.delay = delay;
    }
}

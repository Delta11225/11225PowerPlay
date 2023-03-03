package org.firstinspires.ftc.teamcode.util.types;

public enum LinearSlideMode {
    MANUAL(0),
    GROUND(1),
    LOW(2),
    MEDIUM(3),
    HIGH(4);

    public final int val;
    LinearSlideMode(int val) {
        this.val = val;
    }
    }

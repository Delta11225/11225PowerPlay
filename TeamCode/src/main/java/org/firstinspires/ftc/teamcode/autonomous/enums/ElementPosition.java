package org.firstinspires.ftc.teamcode.autonomous.enums;

public enum ElementPosition {
    MIDDLE("middle"),
    LEFT("left"),
    RIGHT("right")
    ;

    private final String text;

    ElementPosition(final String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return text;
    }
}

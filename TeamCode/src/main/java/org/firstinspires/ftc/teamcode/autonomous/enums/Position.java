package org.firstinspires.ftc.teamcode.autonomous.enums;

public enum Position {
    FRONT("front"),
    BACK("back")
    ;

    private final String text;

    Position(final String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return text;
    }
}

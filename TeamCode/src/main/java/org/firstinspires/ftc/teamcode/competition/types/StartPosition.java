package org.firstinspires.ftc.teamcode.competition.types;

public enum StartPosition {
    FRONT("front"),
    BACK("back")
    ;

    private final String text;

    StartPosition(final String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return text;
    }
}

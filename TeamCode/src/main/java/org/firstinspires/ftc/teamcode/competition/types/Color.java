package org.firstinspires.ftc.teamcode.competition.types;

public enum Color {
    RED("red"),
    BLUE("blue")
    ;

    private final String text;

    Color(final String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return text;
    }
}

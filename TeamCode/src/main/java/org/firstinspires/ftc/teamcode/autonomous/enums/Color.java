package org.firstinspires.ftc.teamcode.autonomous.enums;

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

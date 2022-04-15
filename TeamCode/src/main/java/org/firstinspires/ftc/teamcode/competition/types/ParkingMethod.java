package org.firstinspires.ftc.teamcode.competition.types;

public enum ParkingMethod {
    WALL("wall"),
    BARRIER("barrier"),
    STORAGE("storage")
    ;

    private final String text;

    ParkingMethod(final String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return text;
    }
}

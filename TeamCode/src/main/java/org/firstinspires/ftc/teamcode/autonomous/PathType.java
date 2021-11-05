package org.firstinspires.ftc.teamcode.autonomous;

import java.util.HashMap;
import java.util.Map;

public enum PathType {
    LINE_TO_CONSTANT(0),
    LINE_TO_LINEAR(1),
    SPLINE_TO_LINEAR(2);

    private final int value;
    private static final Map map = new HashMap<>();

    PathType(int value) {
        this.value = value;
    }

    static {
        for (PathType pageType : PathType.values()) {
            map.put(pageType.value, pageType);
        }
    }

    public static PathType valueOf(int pageType) {
        return (PathType) map.get(pageType);
    }

    public int getValue() {
        return value;
    }
}

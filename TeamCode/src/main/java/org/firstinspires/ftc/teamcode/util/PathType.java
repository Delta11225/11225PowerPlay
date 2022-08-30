package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;
import java.util.Map;

public enum PathType {
    LINE_TO_LINEAR(0),
    LINE_TO_SPLINE(1),
    SPLINE_TO_LINEAR(2),
    SPLINE_TO_SPLINE(3),

    // FIXME remove when working on trajectory generator
    // Legacy support, remove later
    LINE_TO_CONSTANT(-999);

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

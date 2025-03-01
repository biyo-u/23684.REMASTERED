package org.firstinspires.ftc.teamcode.Old.Utilities.Old;

import org.jetbrains.annotations.NotNull;

public class Vector2D {
    private Distance x;
    private Distance y;

    // Make this take a DistanceValue
    public Vector2D(Distance x, Distance y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x.getDistance();
    }

    public double getY() {
        return y.getDistance();
    }

    public void setX(Distance x) {
        this.x = x;
    }

    public void setY(Distance y) {
        this.y = y;
    }

    public boolean equals(@NotNull Vector2D obj) {
        return x == obj.x && y == obj.y;
    }
}
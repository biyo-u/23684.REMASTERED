package org.firstinspires.ftc.teamcode.Old.Utilities.Old;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

public class Position {

    private Rotation rotation;
    private Vector2D vector2D;

    public Position(Distance x, Distance y, Rotation rotation) {
        this.rotation = rotation;
        this.vector2D = new Vector2D(x, y);
    }

    public Rotation getRotation() {
        return rotation;
    }

    public void setRotation(Rotation rotation) {
        this.rotation = rotation;
    }

    public Vector2D getVector2D() {
        return vector2D;
    }

    public double getX() {
        return vector2D.getX();
    }

    public double getY() {
        return vector2D.getY();
    }

    public double getHeading() {
        return rotation.getAngle();
    }

    public void setX(Distance x) {
        vector2D.setX(x);
    }

    public void setY(Distance y) {
        vector2D.setY(y);
    }

    public void setHeading(double heading) {
        rotation.setAngle(heading, AngleUnit.DEGREES);
    }

    public void setVector2D(Vector2D vector2D) {
        this.vector2D = vector2D;
    }

    public boolean equals(@NotNull Position obj) {
        return rotation.equals(obj.rotation) && vector2D.equals(obj.vector2D);
    }
}

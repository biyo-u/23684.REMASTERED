package org.firstinspires.ftc.teamcode.Old.Utilities.Old;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Rotation {
    double angle;

    public Rotation(double angle, AngleUnit unit) {
        if (unit == AngleUnit.DEGREES) {
            this.angle = angle;
        } else if (unit == AngleUnit.RADIANS){
            this.angle = Math.toDegrees(angle);
        }
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle, AngleUnit unit) {
        if (unit == AngleUnit.DEGREES) {
            this.angle = angle;
        } else if (unit == AngleUnit.RADIANS){
            this.angle = Math.toDegrees(angle);
        }
    }
}
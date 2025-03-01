package org.firstinspires.ftc.teamcode.Old.Utilities.Old;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance {
    private double distance;

    public Distance(double distance, DistanceUnit unit){
        if (unit == DistanceUnit.INCH) {
            this.distance = distance;
        } else if (unit == DistanceUnit.METER) {
            this.distance = distance * 39.37007874015748031496062992125984251968503937007874015748031496062992125984251968503937007874015748031496062992125984251968503937007874015748031496062992125984251968503937007874015748031496062992126;
        } else if (unit == DistanceUnit.MM) {
            this.distance = distance / 25.4;
        } else if (unit == DistanceUnit.CM) {
            this.distance = distance / 2.54;
        }
    }

    public double getDistance(){
        return distance;
    }

    public void setDistance(double distance, DistanceUnit unit){
        if (unit == DistanceUnit.INCH) {
            this.distance = distance;
        } else if (unit == DistanceUnit.METER) {
            this.distance = distance * 39.37007874015748031496062992125984251968503937007874015748031496062992125984251968503937007874015748031496062992125984251968503937007874015748031496062992125984251968503937007874015748031496062992126;
        } else if (unit == DistanceUnit.MM) {
            this.distance = distance / 25.4;
        } else if (unit == DistanceUnit.CM) {
            this.distance = distance / 2.54;
        }
    }
}
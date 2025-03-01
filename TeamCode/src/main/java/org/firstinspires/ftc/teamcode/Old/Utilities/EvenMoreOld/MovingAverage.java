package org.firstinspires.ftc.teamcode.Old.Utilities.EvenMoreOld;

import java.util.LinkedList;

// Kalman Filter would be better; are there already utility classes for this?
// TODO: change to a kalman filter in future
public class MovingAverage {

    public int desired_samples;
    public LinkedList<Double> samples;

    public MovingAverage(int samples) {
        this.desired_samples = samples;
        this.samples = new LinkedList();
    }

    public void add_sample(double value) {
        samples.add(0,new Double(value));
        while (samples.size() > desired_samples) {
            samples.removeLast();
        }
    }

    public double current_value() {
        double total = 0.0;
        int number = 0;
        for (Double v : samples) {
            total += v.doubleValue();
            number++;
        }
        if (number == 0) return 0.0;
        return total / number;
    }
}

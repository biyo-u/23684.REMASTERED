package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Utilities.Robot;


public class PIDDriver {

    // TODO: FIND VALUES!!!!
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double setpoint = 0;

    public Robot robot;

    // Creates a PIDFController with gains kP, kI, kD, and kF
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public PIDDriver(double target) {
        this.setpoint = target;

        // set our gains to some value
        pidf.setP(0.37);
        pidf.setI(0.05);
        pidf.setD(1.02);

        // get our gain constants
        kP = pidf.getP();
        kI = pidf.getI();
        kD = pidf.getD();

        // set all gains
        pidf.setPIDF(kP, kI, kD, 0.7);

        // get all gain coefficients

        double[] coefficients = pidf.getCoefficients();
        kP = coefficients[0];
        kI = coefficients[1];
        kD = coefficients[2];
        kF = coefficients[3];
    }

    public double getPIDOutput(double position){
        // Calculates the output of the PIDF algorithm based on sensor
        // readings. Requires both the measured value
        // and the desired setpoint
        double output = pidf.calculate(
                // TODO: Correct this for our set up
                position, setpoint
        );

        // NOTE: motors have internal PID control
        // Sets the error tolerance to 5, and the error derivative
        // tolerance to 10 per second
        pidf.setTolerance(5, 10);

        // Returns true if the error is less than 5 units, and the
        // error derivative is less than 10 units

        if (pidf.atSetPoint()){
            return 1234567.89;
        } else {
            return output;
        }
    }
}

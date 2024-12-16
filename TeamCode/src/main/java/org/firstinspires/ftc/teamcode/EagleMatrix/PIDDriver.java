package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Utilities.Robot;

@Config
public class PIDDriver {

	// TODO: FIND Kp!!! (by increasing Kp!!!) (try 0.999 or 1.000)!!
	public static double kP = 0.75; // determines force robot moves at
	public static double kI = 0; // do not touch
	public static double kD = 0.946; // determines robot overshoot prevention
	private double kF = 0.7; // do not touch
	private double target; // target value

	private Robot robot;

	// Creates a PIDFController with gains kP, kI, kD, and kF
	PIDFController pidf;

	public PIDDriver(double target) {
		this.target = target;
		this.pidf = new PIDFController(kP, kI, kD, kF);
	}

	public double getP(){
		return pidf.getP();
	}

	public double getI(){
		return pidf.getI();
	}

	public double getD(){
		return pidf.getD();
	}

	public double getPIDOutput(double position){
		double output = pidf.calculate(
				position, target
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

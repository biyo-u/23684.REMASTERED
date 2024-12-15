package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Utilities.Robot;

@Config
public class PIDDriver {

	// TODO: FIND VALUES!!!!
	public double kP = 0;
	public double kI = 0;
	public double kD = 0;
	public double kF = 0;
	private double target;

	private Robot robot;

	// Creates a PIDFController with gains kP, kI, kD, and kF
	PIDFController pidf = new PIDFController(kP, kI, kD, kF);

	public PIDDriver(double target) {
		this.target = target;
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

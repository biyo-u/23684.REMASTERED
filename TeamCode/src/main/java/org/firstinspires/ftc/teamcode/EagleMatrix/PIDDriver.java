package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Utilities.Robot;

@Config
public class PIDDriver {

	// TODO: Fine-tune kP and kD to prevent violent oscillations at movements longer than 48 inches.
	public static double kP = 9.00; // determines force robot moves at
	public static double kI = 0; // do not touch
	public static double kD = 0.946; // determines robot overshoot prevention
	private double kF = 0.7; // do not touch
	private double target; // target value

	public static double angleP = 0; // determines force robot moves at
	public static double angleI = 0; // do not touch
	public static double angleD = 0; // determines robot overshoot prevention
	public static double angleF = 0.7; // do not touch
	private double angleTarget; // target value (angle)

	private Robot robot;

	// Creates a PIDFController with gains kP, kI, kD, and kF
	PIDFController pidf;
	PIDFController anglePidf;

	// TODO: Create separate PID driver for eaglePIDPYaw (turning), which will require a separate set of P, I, and D values. YAY!!!
	public PIDDriver(double target) {
		this.target = target;
		this.pidf = new PIDFController(kP, kI, kD, kF);
		this.anglePidf = new PIDFController(angleP, angleI, angleD, angleF);
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

	public double getAngleP(){
		return anglePidf.getP();
	}
	public double getAngleI(){
		return anglePidf.getI();
	}
	public double getAngleD(){
		return anglePidf.getD();
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

	public double getAnglePIDOutput(double position){
		double angleOutput = anglePidf.calculate(
				position, target
		);

		// NOTE: motors have internal PID control
		// Sets the error tolerance to 5, and the error derivative
		// tolerance to 10 per second
		anglePidf.setTolerance(5, 10);

		// Returns true if the error is less than 5 units, and the
		// error derivative is less than 10 units
		if (anglePidf.atSetPoint()){
			return 1234567.89;
		} else {
			return angleOutput;
		}
	}
}

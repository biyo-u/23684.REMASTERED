package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

public class Lift {
	private final DcMotor liftMotorLeft;
	private final DcMotor liftMotorRight;
	private final DcMotor shoulder;

	/**
	 * Constructor for the Lift class.
	 * <p>
	 * Initializes the lift motors and sets their zero power behavior and run mode.
	 *
	 * @param liftMotorLeft      The left lift motor.
	 * @param liftMotorRight     The right lift motor.
	 */
	public Lift(DcMotor liftMotorLeft, DcMotor liftMotorRight, DcMotor shoulder) {
		this.liftMotorLeft = liftMotorLeft;
		this.liftMotorRight = liftMotorRight;
		this.shoulder = shoulder;
		this.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		this.liftMotorRight.setDirection(DcMotor.Direction.FORWARD);
		this.liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	/**
	 * Controls the lift motors to move the lift.
	 *
	 * @param speed The speed at which to run the lift motors.
	 *              Positive values move the lift up, negative values move it down.
	 */
	// TODO: IMPORTANT Add threshold for lift
	public void liftMove(double speed) {
		liftMotorLeft.setPower(speed);
		liftMotorRight.setPower(speed);
	}

	/**
	 * Controls the movement of the shoulder motor.
	 * <p>
	 * This method sets the power of the shoulder motor based on the provided speed,
	 * while respecting the defined forward and backward limits.
	 * If the speed is positive, the motor moves forward until it reaches the forward limit.
	 * If the speed is negative, the motor moves backward until it reaches the backward limit.
	 * If the speed is zero, the motor stops.
	 *
	 * @param speed The desired speed of the shoulder motor.
	 *              Positive values move the motor forward,
	 *              negative values move it backward,
	 *              and zero stops the motor.
	 */
	// TODO: IMPORTANT Add Thresholds
	public void shoulderMove(double speed) {
		shoulder.setPower(speed);
	}

	public DcMotor getLiftMotorLeft(){
		return liftMotorLeft;
	}

	public DcMotor getLiftMotorRight(){
		return liftMotorLeft;
	}

	public String getJointLiftPosition() {
		return String.format(Locale.getDefault(), """
                Joint Lift Position: %d""", (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition()) / 2);
	}
	public double getLiftPosition() {
		return (double) (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition()) / 2;
	}
	public double getShoulderPosition() {
		return (shoulder.getCurrentPosition());
	}

	public String getTelemetry() {
		return String.format(Locale.getDefault(), """
                Lift Motor Left: %d
                Lift Motor Right: %d
                Shoulder Motor: %d""", liftMotorLeft.getCurrentPosition(), liftMotorRight.getCurrentPosition(), shoulder.getCurrentPosition());
	}
}
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

public class Intake {
	private final Servo clawFront;
	private final Servo wristServo;
	private final Servo clawBack;
	private final Servo spinWrist;
	private final DcMotor elbowMotor;
	private final Servo wristServoBack;

	/**
	 * Constructor for the Intake subsystem.
	 *
	 * @param clawFront The Servo object representing the claw servo.
	 * @param wristServo The servo object representing the claw lift.
	 * @param clawBack The spinning claw servo.
	 * @param wristServoBack The rear wrist Servo.
	 */
	public Intake(Servo clawFront, Servo wristServo, Servo clawBack, DcMotor elbowMotor, Servo spinWrist, Servo wristServoBack) {
		this.clawFront = clawFront; // front claw
		this.wristServo = wristServo; // front claw's wrist
		this.clawBack = clawBack; // back claw
		this.elbowMotor = elbowMotor; // elbow
		this.spinWrist = spinWrist; // back claw spin
		this.wristServoBack = wristServoBack; // back claw wrist
	}

	/**
	 * Opens the claw of the robot.
	 * <p>
	 * This method sets the position of the claw servo to 0, which corresponds to the open position.
	 */
	public void clawOpen() {
		clawFront.setPosition(0);
	}

	/**
	 * Closes the claw of the robot.
	 * <p>
	 * This method sets the position of the claw servo to 1, which corresponds to the closed position.
	 */
	public void clawClose() {
		clawFront.setPosition(1);
	}

	/**
	 * Moves the wrist servo to the "up" position.
	 * <p>
	 * This method sets the position of the wrist servo to 1.
	 */
	public void wristUp() {
		wristServo.setPosition(1);
	}

	/**
	 * Moves the wrist servo to the "down" position.
	 * <p>
	 * This method sets the position of the wrist servo to 0.
	 */
	public void wristDown() {
		wristServo.setPosition(0);
	}

	public void spinWristSideways(){
		spinWrist.setPosition(1);
	}

	public void spinWristStraight(){
		spinWrist.setPosition(0);
	}

	public void wristBackDown(){
		wristServoBack.setPosition(1);
	}

	public void wristBackUp(){
		wristServoBack.setPosition(0);
	}

	public void clawBackClose(){
		clawBack.setPosition(1);
	}

	public void clawBackOpen(){
		clawBack.setPosition(0);
	}

	public void ElbowMotorDown(){
		elbowMotor.setPower(0.5);
	}

	public void ElbowMotorUp(){
		elbowMotor.setPower(-0.5);
	}

	public String getTelemetry() {
		return String.format(Locale.getDefault(), """
                Front Claw Servo: %f
                Wrist Servo: %f
                Spin Wrist Servo: %f
                Claw Back Servo: %f""", clawFront.getPosition(), wristServo.getPosition(), spinWrist.getPosition(), clawBack.getPosition());
	}
}

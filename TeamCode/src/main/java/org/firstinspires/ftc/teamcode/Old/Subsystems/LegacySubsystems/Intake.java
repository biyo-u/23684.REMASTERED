package org.firstinspires.ftc.teamcode.Old.Subsystems.LegacySubsystems;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

public class Intake {
	private final Servo claw;
	private final Servo wrist;

	/**
	 * Constructor for the Intake subsystem.
	 *
	 * @param claw The Servo object representing the claw servo.
	 * @param wrist The servo object representing the claw lift.
	 */
	public Intake(Servo claw, Servo wrist) {
		this.claw = claw; // claw
		this.wrist = wrist; // claw's wrist
	}

	/**
	 * Opens the claw of the robot.
	 * <p>
	 * This method sets the position of the claw servo to 0, which corresponds to the open position.
	 */
	public void clawOpen() {
		claw.setPosition(0);
	}

	/**
	 * Closes the claw of the robot.
	 * <p>
	 * This method sets the position of the claw servo to 1, which corresponds to the closed position.
	 */
	public void clawClose() {
		claw.setPosition(1);
	}

	/**
	 * Moves the wrist servo to the "up" position.
	 * <p>
	 * This method sets the position of the wrist servo to 1.
	 */
	public void wristUp() {
		wrist.setPosition(1);
	}

	/**
	 * Moves the wrist servo to the "down" position.
	 * <p>
	 * This method sets the position of the wrist servo to 0.
	 */
	public void wristDown() {
		wrist.setPosition(0);
	}

	public String getTelemetry() {
		return String.format(Locale.getDefault(), """
                Front Claw Servo: %f
                Wrist Servo: %f""", claw.getPosition(), wrist.getPosition());
	}
}

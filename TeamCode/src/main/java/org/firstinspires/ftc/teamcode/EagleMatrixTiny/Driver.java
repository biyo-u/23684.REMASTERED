package org.firstinspires.ftc.teamcode.EagleMatrixTiny;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;

public class Driver {
	Pose2D currentPosition;
	Pose2D targetPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
	double DISTANCE_THRESHOLD;
	private double xPower = 0;
	private double yPower = 0;
	private double headingPower = 0;
	private double direction = 0; // 0 is Strafe, 1 is Back and Forth, 2 is Turn
	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor rearLeft;
	DcMotor rearRight;
	GoBildaPinpointDriver odo;
	Telemetry telemetry;

	public Driver(HardwareMap hardwareMap, Telemetry telemetry){
		this.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
		this.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
		this.rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
		this.rearRight = hardwareMap.get(DcMotor.class, "rearRight");
		this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
		this.telemetry = telemetry;


		this.odo.setOffsets(-173.0, -156); //measured in mm
		this.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
		this.odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		this.odo.resetPosAndIMU();

		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	public void moveToX(double targetX, double DISTANCE_THRESHOLD){
		this.targetPosition = new Pose2D(DistanceUnit.INCH, targetX, targetPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES, targetPosition.getHeading(AngleUnit.DEGREES));
		this.DISTANCE_THRESHOLD = DISTANCE_THRESHOLD;
		this.direction = 0;
	}

	public void moveToY(double targetY, double DISTANCE_THRESHOLD){
		this.targetPosition = new Pose2D(DistanceUnit.INCH, targetPosition.getX(DistanceUnit.INCH), targetY, AngleUnit.DEGREES, targetPosition.getHeading(AngleUnit.DEGREES));
		this.DISTANCE_THRESHOLD = DISTANCE_THRESHOLD;
		this.direction = 1;
	}

	public void turnTo(double targetHeading, double DISTANCE_THRESHOLD){
		this.targetPosition = new Pose2D(DistanceUnit.INCH, targetPosition.getX(DistanceUnit.INCH), targetPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES, targetHeading);
		this.DISTANCE_THRESHOLD = DISTANCE_THRESHOLD;
		this.direction = 1;
	}

	public boolean update(){
		odo.update();
		currentPosition = odo.getPosition();

		telemetry.addData("CURRENT POSITION X", currentPosition.getX(DistanceUnit.INCH));
		telemetry.addData("CURRENT POSITION Y", currentPosition.getY(DistanceUnit.INCH));
		telemetry.addData("CURRENT POSITION HEADING", currentPosition.getHeading(AngleUnit.DEGREES));

		telemetry.addData("DIRECTION", direction);

		// Move X (Strafe)
		if (direction == 0) {
			if (Math.abs(currentPosition.getX(DistanceUnit.INCH) - targetPosition.getX(DistanceUnit.INCH)) <= DISTANCE_THRESHOLD) {
				// Reached X
				telemetry.addLine("Reached required X position");
				xPower = 0;
			} else if (currentPosition.getX(DistanceUnit.INCH) > targetPosition.getX(DistanceUnit.INCH)) {
				// Move back
				telemetry.addLine("Need to move back to reach required X position");
				xPower = -1;
			} else if (currentPosition.getX(DistanceUnit.INCH) < targetPosition.getX(DistanceUnit.INCH)) {
				// Move forward
				telemetry.addLine("Need to move forward to reach required X position");
				xPower = 1;
			}
		} else {
			xPower = 0;
		}

		// Move Y (Back and Forth)
		if (direction == 1) {
			if (Math.abs(currentPosition.getY(DistanceUnit.INCH) - targetPosition.getY(DistanceUnit.INCH)) <= DISTANCE_THRESHOLD) {
				// Reached Y
				yPower = 0;
			} else if (currentPosition.getY(DistanceUnit.INCH) > targetPosition.getY(DistanceUnit.INCH)) {
				// Move back
				yPower = -1;
			} else if (currentPosition.getY(DistanceUnit.INCH) < targetPosition.getY(DistanceUnit.INCH)) {
				// Move forward
				yPower = 1;
			}
		} else {
			yPower = 0;
		}

		// Turn
		if (direction == 2) {
			if (Math.abs(currentPosition.getHeading(AngleUnit.DEGREES) - targetPosition.getHeading(AngleUnit.DEGREES)) <= DISTANCE_THRESHOLD) {
				// Reached Y
				headingPower = 0;
			} else if (currentPosition.getHeading(AngleUnit.DEGREES) > targetPosition.getHeading(AngleUnit.DEGREES)) {
				// Move back
				headingPower = -1;
			} else if (currentPosition.getHeading(AngleUnit.DEGREES) < targetPosition.getHeading(AngleUnit.DEGREES)) {
				// Move forward
				headingPower = 1;
			}
		} else {
			headingPower = 0;
		}

		double x = xPower * 0.4;
		double y = yPower * 0.4;
		double rx = headingPower * 0.4;

		double botHeading = odo.getHeading();

		// Rotate the movement direction counter to the bot's rotation
		double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
		double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

		rotX = rotX * 1.1;  // Counteract imperfect strafing

		// Denominator is the largest motor power (absolute value) or 1
		// This ensures all the powers maintain the same ratio,
		// but only if at least one is out of the range [-1, 1]
		double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
		double frontLeftPower = (rotY + rotX + rx) / denominator;
		double backLeftPower = (rotY - rotX + rx) / denominator;
		double frontRightPower = (rotY - rotX - rx) / denominator;
		double backRightPower = (rotY + rotX - rx) / denominator;

		frontLeft.setPower(frontLeftPower);
		rearLeft.setPower(backLeftPower);
		frontRight.setPower(frontRightPower);
		rearRight.setPower(backRightPower);

		// Reached position or not
		return xPower == 0 && yPower == 0 && headingPower == 0;
	}
}

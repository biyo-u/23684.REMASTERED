package org.firstinspires.ftc.teamcode.EagleMatrix.Tasks;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.EagleMatrix.PIDDriver;
import org.firstinspires.ftc.teamcode.Utilities.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Task;


public class DrivePIDTask extends Task {
	PIDDriver PIDXDriver;
	PIDDriver PIDYDriver;
	PIDDriver PIDDriverYaw;
	Robot robot;

	public DrivePIDTask(Pose2D target, Robot robot) {
		this.robot = robot;
		PIDXDriver = new PIDDriver(target.getX(DistanceUnit.INCH));
		PIDYDriver = new PIDDriver(target.getY(DistanceUnit.INCH));
		PIDDriverYaw = new PIDDriver(target.getHeading(AngleUnit.DEGREES));
	}

	@Override
	public boolean run() {
		robot.odometry.update();

		// loops this until it returns true
		double eaglePIDXValue = PIDXDriver.getPIDOutput(robot.odometry.getPosition().getX(DistanceUnit.INCH));
		double eaglePIDYValue = PIDYDriver.getPIDOutput(robot.odometry.getPosition().getY(DistanceUnit.INCH));
		double eaglePIDYawValue = PIDDriverYaw.getPIDOutput(robot.odometry.getPosition().getHeading(AngleUnit.DEGREES));

		if (eaglePIDXValue == 1234567.89 && eaglePIDYValue == 1234567.89 && eaglePIDYawValue == 1234567.89){
			// stops robot if PID has reached target
			robot.telemetry.addData("PID X Value", "0");
			robot.telemetry.addData("PID Y Value", "0");
			robot.telemetry.addData("PID Heading Value", "0");
			robot.drive.driveMecanumFieldCentric(0,0,0);
			return true;
		} else {
			if (eaglePIDXValue == 1234567.89){
				eaglePIDXValue = 0;
			}
			if (eaglePIDYValue == 1234567.89){
				eaglePIDYValue = 0;
			}
			if (eaglePIDYawValue == 1234567.89){
				eaglePIDYawValue = 0;
			}

			robot.telemetry.addData("Odometry X Value", robot.odometry.getPosition().getX(DistanceUnit.INCH));
			robot.telemetry.addData("Odometry Y Value", robot.odometry.getPosition().getY(DistanceUnit.INCH));
			robot.telemetry.addData("Odometry Heading Value", robot.odometry.getPosition().getHeading(AngleUnit.DEGREES));
			robot.telemetry.addData("Power X Value", eaglePIDXValue);
			robot.telemetry.addData("Power Y Value", eaglePIDYValue);
			robot.telemetry.addData("Power Heading Value", eaglePIDYawValue);
			robot.telemetry.addData("XP", PIDXDriver.getP());
			robot.telemetry.addData("XI", PIDXDriver.getI());
			robot.telemetry.addData("XD", PIDXDriver.getD());
			robot.telemetry.addData("YP", PIDYDriver.getP());
			robot.telemetry.addData("YI", PIDYDriver.getI());
			robot.telemetry.addData("YD", PIDYDriver.getD());
			robot.telemetry.addData("HP", PIDDriverYaw.getP());
			robot.telemetry.addData("HI", PIDDriverYaw.getI());
			robot.telemetry.addData("HD", PIDDriverYaw.getD());
			robot.drive.driveMecanumFieldCentric(-eaglePIDYValue, eaglePIDXValue, eaglePIDYawValue);
			return false;
		}
	}
}

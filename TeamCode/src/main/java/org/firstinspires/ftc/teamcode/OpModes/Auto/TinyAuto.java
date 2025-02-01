package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.EagleMatrixTiny.Driver;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

@Autonomous(name = "TinyAuto", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class TinyAuto extends OpMode {
	Driver driver;
	boolean isStopped;

	@Override
	public void init() {
		this.driver = new Driver(hardwareMap, telemetry);
	}
	@Override
	public void loop() {
		// POSITION 1
		this.driver.moveToX(24, 2);
		while (!driver.update() && !isStopped) {}

		// POSITION 2
//		this.driver.moveTo(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0), 2);
//		while (!driver.update()) {
//
//		}
	}

	@Override
	public void stop() {
		isStopped = true; // Mark the OpMode as stopped
		telemetry.addData("Status", "OpMode Stopped");
		telemetry.update();
	}
}

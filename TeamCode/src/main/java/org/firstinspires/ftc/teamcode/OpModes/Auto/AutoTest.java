package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.EagleMatrixTiny.Driver;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

@Autonomous(name = "AutoTest", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class AutoTest extends OpMode {
	Driver driver;
	boolean isStopped;

	@Override
	public void init() {
		this.driver = new Driver(hardwareMap, telemetry);
	}
	@Override
	public void loop() {
		// Move Forward
		this.driver.moveToY(10,2);
		while (!driver.update() && !isStopped) {}

		//Move Backward
		this.driver.moveToY(-10, 2);
		while (!driver.update() && !isStopped) {}

		//Move Left
		this.driver.moveToX(-10, 2);
		while (!driver.update() && !isStopped) {}

		// Move Right
		this.driver.moveToX(10, 2);
		while (!driver.update() && !isStopped) {}

		// Turning Clockwise
		this.driver.turnTo(90, 2);
		while (!driver.update() && !isStopped) {}

		// Turning Clockwise
		this.driver.turnTo(-90, 2);
		while (!driver.update() && !isStopped) {}
	}

	@Override
	public void stop() {
		isStopped = true; // Mark the OpMode as stopped
		telemetry.addData("Status", "OpMode Stopped");
		telemetry.update();
	}
}

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.EagleMatrixTiny.Driver;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

@Autonomous(name = "Advanced Auto", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class AdvancedAuto extends OpMode {
	Driver driver;
	boolean isStopped;

	@Override
	public void init() {
		this.driver = new Driver(hardwareMap, telemetry);
	}
	@Override
	public void loop() {
		this.driver.moveToY(12, 2);
		while (!driver.update() && !isStopped) {}

		this.driver.moveToY(-36, 2);
		while (!driver.update() && !isStopped) {}

		this.driver.turnTo(-135, 2);
		while (!driver.update() && !isStopped) {}
	}

	@Override
	public void stop() {
		isStopped = true; // Mark the OpMode as stopped
		telemetry.addData("Status", "OpMode Stopped");
		telemetry.update();
	}
}

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.EagleMatrixTiny.Driver;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

@Autonomous(name = "Basic Auto", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class BasicAuto extends OpMode {
	Driver driver;
	boolean isStopped;

	@Override
	public void init() {
		this.driver = new Driver(hardwareMap, telemetry);
	}
	@Override
	public void loop() {
		// Park
		this.driver.moveToX(36, 2);
		while (!driver.update() && !isStopped) {}
	}

	@Override
	public void stop() {
		isStopped = true; // Mark the OpMode as stopped
		telemetry.addData("Status", "OpMode Stopped");
		telemetry.update();
	}
}

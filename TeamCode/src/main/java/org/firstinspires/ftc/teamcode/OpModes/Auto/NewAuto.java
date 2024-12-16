package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.EagleMatrix.Tasks.DrivePIDTask;
import org.firstinspires.ftc.teamcode.Utilities.Action;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriver;
import org.firstinspires.ftc.teamcode.Utilities.AutoActions;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

@Config
@TeleOp(name="Auto", group= Constants.GroupNames.Autonomous)
public class NewAuto extends OpMode {
	private AutoActions autoActions;
	private AutoDriver autoDriver;
	public static double distance = 24;

	@Override
	public void init() {
		Robot robot = new Robot(hardwareMap, telemetry);

		autoActions = new AutoActions().add(new Action(new DrivePIDTask(new Pose2D(DistanceUnit.INCH, 0, distance, AngleUnit.DEGREES, 0), robot), "FIRST_DRIVE", "0"));
		autoDriver = new AutoDriver(autoActions);
	}

	@Override
	public void loop() {
		autoDriver.run();
	}
}
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
@TeleOp(name="turnAuto", group= Constants.GroupNames.Autonomous)
public class tuninganglePID extends OpMode {
    private AutoActions autoActions;
    private AutoDriver autoDriver;
    public static double displacement = 0;
    public static double distance = 0;
    public static double theta = 180;

    @Override
    public void init() {
        Robot robot = new Robot(hardwareMap, telemetry);
        autoActions = new AutoActions().add(new Action(new DrivePIDTask(new Pose2D(DistanceUnit.INCH, displacement, distance, AngleUnit.DEGREES, theta), robot), "ANGLE_PID_TUNING", "0"));
        autoDriver = new AutoDriver(autoActions);
    }

    @Override
    public void loop() {
        autoDriver.run();
    }
}

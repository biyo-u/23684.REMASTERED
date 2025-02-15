package org.firstinspires.ftc.teamcode.EagleMatrix.NEW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

/** @noinspection unused*/
@Autonomous(name = "TEST_botPID", group = Constants.GroupNames.Testing)
public class TEST_botPID extends OpMode {
    // SOFTWARE
    botPID bot;

    // HARDWARE
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry);
        bot = new botPID(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.setArmTarget(700);
        bot.setLiftTarget(0);
        bot.setXTarget(0);
        bot.setYTarget(0);
        bot.setHeadingTarget(0);
        bot.setDriveTarget(0,0,0);
    }

    @Override
    public void loop() {
        telemetry.update();
        bot.runArm();
        bot.runLift();
        bot.runDrive();
        bot.getTelemetry();
        telemetry.update();
    }
}

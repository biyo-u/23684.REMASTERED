package org.firstinspires.ftc.teamcode.EagleMatrix.NEW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

/** @noinspection unused*/
public class TEST_botPID extends OpMode {
    // SOFTWARE
    botPID bot;

    // HARDWARE
    Robot robot;
    DcMotor liftMotorLeft;
    DcMotor liftMotorRight;
    DcMotor shoulder;
    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry);
        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        bot = new botPID(robot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.setArmTarget(700);
        bot.setLiftTarget(0);
        bot.setXTarget(0);
        bot.setYTarget(0);
        bot.setHeadingTarget(0);
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

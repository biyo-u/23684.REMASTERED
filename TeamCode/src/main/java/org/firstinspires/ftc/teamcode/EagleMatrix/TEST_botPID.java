package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.MoreOld.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.LegacySubsystems.Robot;

/** @noinspection unused*/
@Autonomous(name = "TEST_botPID", group = Constants.GroupNames.Testing)
public class TEST_botPID extends OpMode {
    botPID bot;
    Robot robot;
    botPIDConstants botPIDConstants;

    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry);
        bot = new botPID(robot, botPIDConstants);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.lift.getShoulder().setDirection(DcMotorSimple.Direction.REVERSE);

        // todo: redo or delete test code idk
//        bot.setArmTarget(0);
//        bot.setLiftTarget(0);
//        bot.setXTarget(0);
//        bot.setYTarget(0);
//        bot.setHeadingTarget(0);
//        bot.setDriveTarget(0,0,0);
        robot.lift.getLiftMotorLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.getLiftMotorRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.lift.getShoulder().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.update();
//        bot.runArm();
//        bot.runLift();
//        bot.runDrive();

        telemetry.addData("Arm Position", bot.getArmPosition());
        telemetry.addData("Lift Position", bot.getLiftPosition());
        telemetry.addData("X Position", bot.getOdoPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y Position", bot.getOdoPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Heading Position", bot.getOdoPosition().getHeading(AngleUnit.DEGREES));

        telemetry.addData("\n Arm Target", bot.getArmTarget());
        telemetry.addData("Lift Target", bot.getLiftTarget());
        telemetry.addData("X Target", bot.getXTarget());
        telemetry.addData("Y Target", bot.getYTarget());
        telemetry.addData("Heading Target", bot.getHeadingTarget());
        telemetry.addData("UnNormalized Heading",bot.getUnNormalizedHeading());

        telemetry.addData("\n X Offset", robot.odometry.XOffset());
        telemetry.addData("Y Offset", robot.odometry.YOffset());

        robot.odometry.update();
        telemetry.update();
    }
}
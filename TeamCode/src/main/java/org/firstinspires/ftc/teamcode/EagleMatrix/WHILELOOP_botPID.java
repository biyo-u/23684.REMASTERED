package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.LegacySubsystems.Robot;
import org.firstinspires.ftc.teamcode.EagleMatrix.botPIDConstants.Arm_Constants;
import org.firstinspires.ftc.teamcode.EagleMatrix.botPIDConstants.Lift_Constants;

/** @noinspection unused*/
@Autonomous(name = "WHILELOOP_botPID", group = Constants.GroupNames.Testing)
public class WHILELOOP_botPID extends OpMode {
    botPID bot;
    Robot robot;
    botPIDConstants botPIDConstants;
    boolean loopIsActive = true;
    boolean LiftAtTarget = false;
    boolean functionsRunning = true;

    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry);
        bot = new botPID(robot, botPIDConstants);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.lift.getShoulder().setDirection(DcMotorSimple.Direction.REVERSE);

        //bot.setArmTarget(Arm_Constants.Arm_Score_High_Baskets);
        //bot.setLiftTarget(Lift_Constants.Lift_Score_Chambers);
        bot.setXTarget(0);
        bot.setYTarget(0);
        bot.setHeadingTarget(0);
        bot.setDriveTarget(0,0,0);
        robot.lift.getLiftMotorLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.getLiftMotorRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.lift.getShoulder().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        bot.runArm(Arm_Constants.Arm_Score_High_Baskets);
        bot.runLift(3000);
        // bot.runDrive();

        double errorLift = bot.getLiftController().calculate(bot.getLiftPosition(), bot.getLiftTarget());
        double errorArm = Double.compare(bot.getArmPosition(), bot.getArmTarget());

        if (errorLift <= 0.1 && errorArm == 0){
            LiftAtTarget = true;
        }
        if (LiftAtTarget == true){
            bot.stopLift();
            bot.stopArm();
            LiftAtTarget = false;
            functionsRunning = false;
            if (functionsRunning == false){
                bot.runArm(Arm_Constants.Arm_Home);
                bot.runLift(Lift_Constants.Lift_Home);
            }
        }

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

        telemetry.addData("Lift At Target?", LiftAtTarget);
        telemetry.addData("ERROR LIFT", errorLift);
        telemetry.addData("ERROR ARM", errorArm);
        telemetry.addData("Lift Motor LEFT Power", robot.lift.getLiftMotorLeft().getPower());
        telemetry.addData("Lift Motor RIGHT Power", robot.lift.getLiftMotorRight().getPower());
        telemetry.addData("Arm Power", robot.lift.getShoulder().getPower());

        robot.odometry.update();
        telemetry.update();
    }
}
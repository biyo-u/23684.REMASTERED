package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class PidArm  extends OpMode {

    private PIDController controller;

    public static double p = 0 , i = 0 , d = 0;
    public static double f = 0;
    public static int target = 0;
    public final double ticksInDegree = 700 / 180.0;
    DcMotor shoulder;

    @Override
    public void init(){

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shoulder = hardwareMap.get(DcMotor.class,"shoulder");



  }
    @Override
    public void loop(){

        controller.setPID(p,i,d);

        int armPosition = shoulder.getCurrentPosition();

        double pid = controller.calculate(armPosition,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double power = pid + ff;

        shoulder.setPower(power);

        telemetry.addData("pos ",armPosition);
        telemetry.addData("target ",target);
        telemetry.update();

    }
}

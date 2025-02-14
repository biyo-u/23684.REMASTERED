package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF.TEST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PidfLiftTest extends OpMode {
    private PIDController controllerLeft;
    private PIDController controllerRight;
    public static double Lp = 0 , Li = 0 , Ld = 0;
    public static double Lf = 0;
    public static double Rp = 0 , Ri = 0 , Rd = 0;
    public static double Rf = 0;
    public static int target = 700;
    public final double ticksInDegree = 700 / 180.0;
    DcMotor liftMotorLeft;
    DcMotor liftMotorRight;

    public void init(){

        controllerLeft = new PIDController(Lp,Li,Ld);
        controllerRight = new PIDController(Rp,Ri,Rd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(){
        // LEFT SLIDE
        controllerLeft.setPID(Lp,Li,Ld);
        double posLeft = liftMotorLeft.getCurrentPosition();
        double pidLeft = controllerLeft.calculate(posLeft,target);
        double ffLeft = Math.cos(Math.toRadians(target / ticksInDegree)) * Lf;
        double powerLeft = pidLeft + ffLeft;

        // RIGHT SLIDE
        controllerRight.setPID(Rp,Ri,Rd);
        double posRight = liftMotorRight.getCurrentPosition();
        double pidRight = controllerRight.calculate(posRight,target);
        double ffRight = Math.cos(Math.toRadians(target / ticksInDegree)) * Rf;
        double powerRight = pidRight + ffRight;

        liftMotorLeft.setPower(powerLeft);
        liftMotorRight.setPower(powerRight);

        telemetry.addData(" Slide position LEFT", posLeft);
        telemetry.addData(" Slide position RIGHT", posRight);
        telemetry.addData("target ",target);
        telemetry.update();
    }
}

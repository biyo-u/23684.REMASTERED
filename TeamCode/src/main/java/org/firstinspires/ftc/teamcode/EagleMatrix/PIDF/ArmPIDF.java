package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ArmPIDF {

    private PIDController controller;

    private static double p = 0.09 , i = 0.001 , d = 0.001;
    private static double f = 0;

    private final double ticksInDegree = 700 / 180.0;

    private final int  scoreTarget = 3000;
    private final int initTarget = 0;

     private DcMotor shoulder;

    public ArmPIDF(HardwareMap hw) {

        controller = new PIDController(p,i,d);

        shoulder = hw.get(DcMotor.class,"shoulder");
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
    }



    public void MoveToInit(){
        int armPosition = shoulder.getCurrentPosition();

        double pid = controller.calculate(armPosition,initTarget);
        double ff = Math.cos(Math.toRadians(initTarget / ticksInDegree)) * f;

        double power = pid + ff;

        shoulder.setPower(power);
    }

    public void MoveToScore(){
        int armPosition = shoulder.getCurrentPosition();

        double pid = controller.calculate(armPosition,scoreTarget);
        double ff = Math.cos(Math.toRadians(scoreTarget / ticksInDegree)) * f;

        double power = pid + ff;

        shoulder.setPower(power);

    }
    public void MoveToTarget(int target){
        controller.setPID(p,i,d);

        int armPosition = shoulder.getCurrentPosition();

        double pid = controller.calculate(armPosition,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double power = pid + ff;

        shoulder.setPower(power);
    }
}

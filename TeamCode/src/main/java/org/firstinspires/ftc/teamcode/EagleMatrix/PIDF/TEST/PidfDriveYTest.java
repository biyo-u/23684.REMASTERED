package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF.TEST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

public class PidfDriveYTest extends OpMode {

    private PIDController controller;

    public static double p = 0 , i = 0 , d = 0;
    public static double f = 0;
    public static int target = 700;

    public final double ticksInDegree = 700 / 180.0;
    Robot robot;
    GoBildaPinpointDriver odo;


    public void init(){

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        robot = new Robot(hardwareMap,telemetry);


    }

    public void loop(){

        controller.setPID(p,i,d);

        double Ypos = odo.getPosY();

        double  Ypid = controller.calculate(Ypos,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double power = Ypid + ff;

        robot.drive.driveMecanumFieldCentric(power,0,0);

        telemetry.addData("Y position ", Ypos);
        telemetry.addData("target Y ",target);
        telemetry.update();
        odo.update();

    }
}

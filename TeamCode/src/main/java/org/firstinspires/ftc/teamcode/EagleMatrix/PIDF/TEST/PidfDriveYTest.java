package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF.TEST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    Pose2D currentPos;

    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        robot = new Robot(hardwareMap,telemetry);

        currentPos = odo.getPosition();
    }

    public void loop(){
        controller.setPID(p,i,d);

        double Y_POSITION = currentPos.getY(DistanceUnit.INCH);

        double Ypid = controller.calculate(Y_POSITION,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double Y_POWER = Ypid + ff;

        // TODO: test to see if FieldCentric is better than Robot Centric
        // robot.drive.driveMecanumFieldCentric(Y_POWER,0,0);
        robot.drive.driveMecanumRobotCentric(Y_POWER,0,0);

        telemetry.addData("Y position ", Y_POSITION);
        telemetry.addData("target Y ",target);
        telemetry.update();
        odo.update();
    }
}

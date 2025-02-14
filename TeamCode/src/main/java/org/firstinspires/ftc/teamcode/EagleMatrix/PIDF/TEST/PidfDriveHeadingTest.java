package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF.TEST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

public class PidfDriveHeadingTest extends OpMode {
    private PIDController controller;

    public static double p = 0 , i = 0 , d = 0;
    public static double f = 0;
    public static int target = 180;

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

        double HEADING_POSITION = currentPos.getHeading(AngleUnit.DEGREES);


        double Hpid = controller.calculate(HEADING_POSITION,target);
        double Hff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double H_POWER = Hpid + Hff;

        // TODO: test to see if FieldCentric is better than Robot Centric
        // robot.drive.driveMecanumFieldCentric(0,0,H_POWER);
        robot.drive.driveMecanumRobotCentric(0,0,H_POWER);

        telemetry.addData("Heading position ", HEADING_POSITION);
        telemetry.addData("target Heading",target);
        telemetry.update();
        odo.update();
    }
}

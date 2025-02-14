package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF.TEST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Old.Distance;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

public class PidfDriveXTest extends OpMode {
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
        double X_POSITION = currentPos.getX(DistanceUnit.INCH);

        double X_PID = controller.calculate(X_POSITION,target);
        double Xff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double X_POWER = X_PID + Xff;

        // TODO: test to see if FieldCentric is better than Robot Centric
        // robot.drive.driveMecanumFieldCentric(0,X_POWER,0);
        robot.drive.driveMecanumRobotCentric(0,X_POWER,0);

        telemetry.addData("X position ", X_POSITION);
        telemetry.addData("target X ",target);
        telemetry.update();
        odo.update();
    }
}

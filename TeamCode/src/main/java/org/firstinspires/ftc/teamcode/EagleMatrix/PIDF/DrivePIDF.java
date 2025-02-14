package org.firstinspires.ftc.teamcode.EagleMatrix.PIDF;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

public class DrivePIDF  {

    public PIDController Xcontroller;
    public PIDController Ycontroller;
    public PIDController Hcontroller;

    Pose2D currentPos;

    // X PIDS
    // TODO: find the  X PIDF for the drive train because thats pretty kewl
    private static double Xp = 0 , Xi = 0 , Xd = 0;
    private static double Xf = 0;
    // Y PIDS
    // TODO: find the Y PIDF for the drive train because "Y" not (ok I'll leave)
    private static double Yp = 0 , Yi = 0 , Yd = 0;
    private static double Yf = 0;

    // Heading PIDS
    // TODO: find the Heading PIDF for the drive train because 你剛剛使用Google翻譯嗎
    private static double Hp = 0 , Hi = 0 , Hd = 0;
    private static double Hf = 0;
    private final double ticksInDegree = 700 / 180.0;

    GoBildaPinpointDriver odo;

    Robot robot;

    public DrivePIDF(HardwareMap hw) {
        Xcontroller = new PIDController(Xp,Xi,Xd);
        Ycontroller = new PIDController(Yp,Yi,Yd);
        Hcontroller = new PIDController(Hp,Hi,Hd);

        robot = new Robot(hw,telemetry);

        currentPos = odo.getPosition();

    }

    public void MigrateTo(double targetX,double targetY, double targetHeading){

        // X PIDF
        Xcontroller.setPID(Xp,Xi,Xd);

        double Xpos = currentPos.getX(DistanceUnit.INCH);

        double  Xpid = Xcontroller.calculate(Xpos,targetX);
        double Xff = Math.cos(Math.toRadians(targetX / ticksInDegree)) * Xf;

        double Xpower = Xpid + Xff;

        // Y PIDF
        Ycontroller.setPID(Yp,Yi,Yd);

        double Ypos = currentPos.getY(DistanceUnit.INCH);

        double  Ypid = Ycontroller.calculate(Ypos,targetY);
        double Yff = Math.cos(Math.toRadians(targetY / ticksInDegree)) * Yf;

        double Ypower = Ypid + Yff;

        // Heading PIDF
        Hcontroller.setPID(Hp,Hi,Hd);

        double Hpos = currentPos.getHeading(AngleUnit.DEGREES);


        double  Hpid = Hcontroller.calculate(Hpos,targetHeading);
        double Hff = Math.cos(Math.toRadians(targetHeading / ticksInDegree)) * Hf;

        double Hpower = Hpid + Hff;

        robot.drive.driveMecanumFieldCentric(Ypower,Xpower,Hpower);



    }



}

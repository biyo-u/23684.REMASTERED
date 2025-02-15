package org.firstinspires.ftc.teamcode.EagleMatrix.NEW;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

import java.util.Locale;

@Config
public class botPID {
    //HARDWARE
    public final Robot robot;

    //PID CONTROLLERS
    public final PIDFController armController;
    public final PIDController liftController;
    public final PIDController driveXController;
    public final PIDController driveYController;
    public final PIDController driveHeadingController;

    // TICKS TO IN-DEGREES
    public final double ArmTicksInDegree = 700 / 180.0; // TODO: FIND CORRECT VALUES FOR ARM, LIFT, DRIVE, or just use it as it
    public final double LiftTicksInDegree = 700 / 180.0;
    public final double DriveTicksInDegree = 700 / 180.0;

    //PID GAINS
    public static double Arm_p = 0.01, Arm_i = 0, Arm_d = 0, Arm_f = 0;
    public static double Lift_p = 0.01, Lift_i = 0, Lift_d = 0, Lift_f = 0;
    public static double Xp = 0.08, Xi = 0, Xd = 0.003, Xf = 0;
    public static double Yp = 0.08, Yi = 0, Yd = 0.009, Yf = 0;
    public static double Heading_p = 0, Heading_i = 0, Heading_d = 0, Heading_f = 0;

    // TARGETS
    public static double Arm_target;
    public static double Lift_target;
    public static double X_target;
    public static double Y_target;
    public static double Heading_target;

    public botPID(Robot robot) {
        this.robot = robot;
        armController = new PIDFController(Arm_p,Arm_i,Arm_d, Arm_f);
        liftController = new PIDController(Lift_p,Lift_i,Lift_d);
        driveXController = new PIDController(Xp,Xi,Xd);
        driveYController = new PIDController(Yp,Yi,Yd);
        driveHeadingController = new PIDController(Heading_p,Heading_i,Heading_d);
    }

    public void setArmTarget(double target){
        Arm_target = target;
    }
    public void setLiftTarget(double target){
        Lift_target = target;
    }
    public void setXTarget(double target){
        X_target = target;
    }
    public void setYTarget(double target){
        Y_target = target;
    }
    public void setHeadingTarget(double target){
        Heading_target = target;
    }
    public void setDriveTarget(double x_target, double y_target, double heading_target){
        X_target = x_target;
        Y_target = y_target;
        Heading_target = heading_target;
    }

    public void runArm(){
        armController.setPIDF(Arm_p,Arm_i,Arm_d,Arm_f);

        double armPosition = robot.lift.getShoulderPosition();

        double armPID = armController.calculate(armPosition, Arm_target);

        double Arm_ff = Math.cos(Math.toRadians(Arm_target / ArmTicksInDegree)) * Arm_f;

        double Arm_power = armPID + Arm_ff;

        robot.lift.getShoulder().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.shoulderMove(Arm_power);
    }
    public void runLift(){
        liftController.setPID(Lift_p,Lift_i,Lift_d);

        double liftPosition = robot.lift.getLiftPosition();

        double liftPID = liftController.calculate(liftPosition, Lift_target);

        double Lift_ff = Math.cos(Math.toRadians(Lift_target / LiftTicksInDegree)) * Lift_f;

        double Lift_power = liftPID + Lift_ff;

        robot.lift.liftMove(Lift_power);
    }
    public void runDrive(){
        driveXController.setPID(Xp,Xi,Xd);
        driveYController.setPID(Yp,Yi,Yd);
        driveHeadingController.setPID(Heading_p,Heading_i,Heading_d);

        double xPosition = robot.odometry.getPosition().getX(DistanceUnit.INCH);
        double yPosition = robot.odometry.getPosition().getY(DistanceUnit.INCH);
        // TODO: Fix with a mod angle that ensures the robot never crosses a 0 (see Craig)
        // TODO: another idea is set variable so whenever angle crosses 0, add 1 (if positive), and minus 1 (if negative)
        double heading = robot.odometry.getPosition().getHeading(AngleUnit.DEGREES);
        // double totalHeading = robot.odometry.getPosition().

        double xPID = driveXController.calculate(xPosition, X_target);
        double yPID = driveYController.calculate(yPosition, Y_target);
        double headingPID = driveHeadingController.calculate(heading, Heading_target);

        double Xff = Math.cos(Math.toRadians(X_target / DriveTicksInDegree)) * Xf;
        double Yff = Math.cos(Math.toRadians(Y_target / DriveTicksInDegree)) * Yf;
        double Heading_ff = Math.cos(Math.toRadians(Heading_target / DriveTicksInDegree)) * Heading_f;

        double X_power = xPID + Xff;
        double Y_power = yPID + Yff;
        double Heading_power = headingPID + Heading_ff;

        robot.drive.driveMecanumFieldCentric(Y_power, X_power, -Heading_power, heading);
    }

    public String getTargets(){

        return String.format(Locale.getDefault(), """
                Arm Target: %f \n
                Lift Target: %f""", Arm_target, Lift_target);
    }
    public double getXTarget(){
        return X_target;
    }
    public double getYTarget(){
        return Y_target;
    }
    public double getHeadingTarget(){
        return Heading_target;
    }

    public double getArmPosition(){
        return robot.lift.getShoulderPosition();
    }

    public double getLiftPosition(){
        return robot.lift.getLiftPosition();
    }
    public Pose2D getOdoPosition(){
        return robot.odometry.getPosition();
    }
}
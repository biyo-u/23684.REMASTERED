package org.firstinspires.ftc.teamcode.Subsystems.LegacySubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;

public class Robot {
    // Public Subsystems
    public Intake intake;
    public Lift lift;
    public OldDrive oldDrive;
    public Compass compass;
    public Odometry odometry;
    public Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Private Devices
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotor liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        DcMotor liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
        DcMotor shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        DcMotor rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        GoBildaPinpointDriver odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Initialize Public Subsystems
        compass = new Compass(odometryComputer);
        intake = new Intake(claw, wrist);
        lift = new Lift(liftMotorLeft, liftMotorRight, shoulder);
        oldDrive = new OldDrive(frontLeft, frontRight, rearLeft, rearRight, compass);
        odometry = new Odometry(odometryComputer, compass);

        // Public Devices (not subsystems)
        this.telemetry = telemetry;
//        this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        this.odometry.setOffsets(-6.44, 6.8745);
//        this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        this.odometry.resetPosAndIMU();
//        this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }
}

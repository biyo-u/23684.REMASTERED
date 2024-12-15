package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Compass;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

public class Robot {
    // Public Subsystems
    public Intake intake;
    public Lift lift;
    public Drive drive;
    public Compass compass;
    public GoBildaPinpointDriver odometry;
    public Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Private Devices
        Servo clawFront = hardwareMap.get(Servo.class, "clawFront");
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo liftServoTilt = hardwareMap.get(Servo.class, "liftServoTilt");
        Servo rightHangServo = hardwareMap.get(Servo.class, "rightHangServo");
        Servo leftHangServo = hardwareMap.get(Servo.class, "leftHangServo");
        Servo clawBack = hardwareMap.get(Servo.class, "clawBack");
        Servo spinWrist = hardwareMap.get(Servo.class, "spinWrist");
        Servo wristBackServo = hardwareMap.get(Servo.class, "wristBackServo");
        DcMotor elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
        DcMotor liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        DcMotor liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
        DcMotor shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        DcMotor rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        GoBildaPinpointDriver odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

        // Initialize Public Subsystems
        compass = new Compass(odometryComputer);
        intake = new Intake(clawFront, wristServo, clawBack, elbowMotor, spinWrist, wristBackServo);
        lift = new Lift(liftMotorLeft, liftMotorRight, shoulderMotor, liftServoTilt, rightHangServo, leftHangServo);
        drive = new Drive(frontLeft, frontRight, rearLeft, rearRight, compass);

        // Public Devices (not subsystems)
        this.telemetry = telemetry;
        this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.odometry.setOffsets(-6.44, 6.8745);
        this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.odometry.resetPosAndIMU();
        this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }
}

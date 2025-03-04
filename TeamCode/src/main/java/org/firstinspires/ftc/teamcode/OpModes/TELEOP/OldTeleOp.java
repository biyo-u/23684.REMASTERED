package org.firstinspires.ftc.teamcode.OpModes.TELEOP;
// TODO: Ensure usage with Command AUTO does not result in issues.
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//
//import org.firstinspires.ftc.teamcode.Old.Utilities.MoreOld.Constants;
//import org.firstinspires.ftc.teamcode.Old.Subsystems.LegacySubsystems.Robot;
//
//import java.util.Locale;
//
//@Disabled
//@TeleOp(name = "TELEOP", group = Constants.GroupNames.TeleOp)
//public class OldTeleOp extends OpMode {
//    private Robot robot;
//
//    @Override
//    public void init() {
//        robot = new Robot(hardwareMap, telemetry);
//    }
//
//    @Override
//    public void loop() {
//        // Drive the robot with the game-pad
//        robot.oldDrive.driveMecanumRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//
//        // Reset IMU for Field Centric
//        if (gamepad1.left_bumper) {
//            robot.compass.resetYaw();
//        }
//
//        // Set speed mode
//        if (gamepad1.left_trigger > 0.8) {
//            robot.oldDrive.setPower(1);
//        } else if (gamepad1.right_trigger > 0.8) {
//            robot.oldDrive.setPower(0.75);
//        } else {
//            robot.oldDrive.setPower(0.5);
//        }
//
//        // Wrist
//        if (gamepad2.x){
//            robot.intake.wristUp();
//        } else if (gamepad2.b) {
//            robot.intake.wristDown();
//        }
//
//        // Lift
//        robot.lift.liftMove(-gamepad2.left_stick_y);
//
//        // Shoulder
//        robot.lift.shoulderMove(-gamepad2.right_stick_y * 0.8);
//
//        // Claw
//        if (gamepad2.right_trigger > 0) {
//            robot.intake.clawOpen();
//        } else if (gamepad2.left_trigger > 0) {
//            robot.intake.clawClose();
//        } else {
//            robot.intake.clawClose();
//        }
//
//        robot.odometry.update();
//
//        // Telemetry
//        telemetry.addLine(robot.lift.getTelemetry());
//        telemetry.addLine(robot.lift.getJointLiftPosition());
//        telemetry.addLine(robot.intake.getTelemetry());
//        Pose2D position = robot.odometry.getPosition();
//        telemetry.addLine(String.format(Locale.getDefault(), "X: %f, Y: %f, Heading: %f", position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH), position.getHeading(AngleUnit.DEGREES)));
//        telemetry.addLine(robot.compass.getTelemetry());
//    }
//}
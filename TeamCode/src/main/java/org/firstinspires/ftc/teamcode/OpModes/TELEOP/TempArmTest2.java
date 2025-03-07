package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Old.Subsystems.LegacySubsystems.Robot;

// TODO: DELETE THIS WHEN IT IS NOT NECESSARY, WHICH IS SOMETHING THAT WILL BE DETERMINED AT A TIME IN THE NEAR FUTURE. TILL THEN, TOUCH THIS NOT!!!

@TeleOp (name = "Arm 2.0 Testing (DELETE LATER)")
public class TempArmTest2 extends OpMode {
    private Robot robot;

    double targetArm = 200;
    double homeArm = 30;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.lift.getShoulder().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.getShoulder().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        // SHOULDER ENCODER USAGE????
        if (gamepad2.y) {
            robot.lift.getShoulder().setTargetPosition((int) targetArm);
            robot.lift.getShoulder().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.getShoulder().setPower(1);
        } else if (gamepad2.a) {
            robot.lift.getShoulder().setTargetPosition((int) homeArm);
            robot.lift.getShoulder().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.getShoulder().setPower(1);
        }

        // CLAW 2
        if (gamepad2.right_trigger > 0) {
            robot.intake.clawOpen();
        } else if (gamepad2.left_trigger > 0) {
            robot.intake.clawClose();
        } else {
            robot.intake.clawClose();
        }

        // WRIST 2
        if (gamepad1.x) {
            robot.intake.wristDown();
        } else if (gamepad1.b) {
            robot.intake.wristUp();
        }

        // SHOULDER 2
        robot.lift.shoulderMove(-gamepad2.right_stick_y * 0.8);

        telemetry.addData("Shoulder Position (Ticks)" , robot.lift.getShoulder().getCurrentPosition());
        telemetry.addData("Shoulder Power Output", robot.lift.getShoulder().getPower());
        telemetry.update();
    }
}
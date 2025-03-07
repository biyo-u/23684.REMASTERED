package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Config
@TeleOp(name = "Mario Kart", group = "Opmode")
public class MarioKart extends OpMode {
    public static double X = 0.0;
    public static double Y = 0.0;
    public static double HEADING = 0.0;
    public static double LIFT = 0.0;
    public static double ANGLE = 0.0;
    GamepadEx controller;
    Drive drive;
    Arm arm;
    Intake intake;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        controller = new GamepadEx(gamepad1);

        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(arm);
        CommandScheduler.getInstance().registerSubsystem(intake);

        drive.reset();
        arm.reset();
        intake.reset();
    }

    @Override
    public void init_loop() {
        drive.readSensors();
        arm.readSensors();
        intake.readSensors();
    }

    @Override
    public void start() {
        drive.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        controller.readButtons();
        drive.readSensors();
        arm.readSensors();
        intake.readSensors();

        if (controller.wasJustPressed(GamepadKeys.Button.X)) {
            CommandScheduler.getInstance().schedule(
                    drive.moveTo(X, Y, HEADING)
            );
        }

        if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            CommandScheduler.getInstance().schedule(
                    arm.liftTo(LIFT)
            );
        }

        if (controller.wasJustPressed(GamepadKeys.Button.B)) {
            CommandScheduler.getInstance().schedule(
                    intake.riseTo(ANGLE)
            );
        }

        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket(false);
        drive.addTelemetry(pack);
        arm.addTelemetry(pack);
        intake.addTelemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        drive.readSensors();
        arm.readSensors();
        drive.stop();
        arm.stop();
        intake.stop();

        CommandScheduler.getInstance().reset();
    }
}


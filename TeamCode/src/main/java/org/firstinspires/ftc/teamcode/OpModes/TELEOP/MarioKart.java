package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mario Kart", group = "Opmode")
public class MarioKart extends OpMode {
    public static double X = 0;
    public static double Y = 0;
    public static double HEADING = 0.0;
    GamepadEx controller;
    Drive drive;
    VoltageSensor battery;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        drive = new Drive(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");
        controller = new GamepadEx(gamepad1);

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);

        drive.reset();
    }

    @Override
    public void init_loop() {
        drive.readSensors();
    }

    @Override
    public void start() {
        drive.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        controller.readButtons();
        drive.readSensors();


        if (controller.wasJustPressed(GamepadKeys.Button.X)) {
            CommandScheduler.getInstance().schedule(
                    drive.moveQuickly(X, Y, HEADING)
            );
        }

        if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                        drive.turnQuickly(X, Y, HEADING)
                    )
            );
        }

        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        drive.addTelemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        drive.readSensors();
        drive.stop();

        // TODO: Add robot's current position here
//        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
//        SharedPreferences.Editor editor = prefs.edit();
//        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
//        editor.apply();

        CommandScheduler.getInstance().reset();
    }
}


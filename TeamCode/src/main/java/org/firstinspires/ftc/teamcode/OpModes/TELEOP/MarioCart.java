package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Mario Kart", group="Opmode")
public class MarioCart extends OpMode {
    GamepadEx controller;
    Drive drive;
    VoltageSensor battery;
    public static double X = 0;
    public static double Y = 0;
    public static double HEADING = 0.0;


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
        drive.read_sensors(time);
    }

    public Command doNothing(long timeout) {
        return new CommandBase() {}.withTimeout(timeout);
    }

    @Override
    public void start() {
        drive.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        controller.readButtons();
        drive.read_sensors(time);


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
        drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
        editor.apply();
        drive.stop();
    }
}


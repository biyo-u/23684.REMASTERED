package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public abstract class TeleOp extends OpMode {
    GamepadEx driver;
    GamepadEx operator;
    VoltageSensor battery;
    Drive drive;
    ElapsedTime runtime = new ElapsedTime();
    boolean endgameAlertTriggered = false;
    boolean parkAlertTriggered = false;

    /**
     * @noinspection unused
     */
    public abstract Alliance getAlliance();

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        drive.reset();
        battery = hardwareMap.voltageSensor.get("Control Hub");

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
    }

    @Override
    public void start() {
        runtime.reset();

        drive.reset();
        drive.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void init_loop() {
        drive.readSensors();
    }

    @Override
    public void loop() {
        driver.readButtons();
        drive.readSensors();

        CommandScheduler.getInstance().run();

        // TODO: REMOVE ALL OF THESE BECAUSE OUR CONTROLLERS DON'T HAVE RUMBLE OR COLORS

        // Check if we're in endgame and trigger gamepad rumble for both drivers
        if (!endgameAlertTriggered && (runtime.seconds() > 90)) {
            Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 250) // Rumble both motors 100% for 250 mSec
                    .build();
            gamepad1.runRumbleEffect(rumbleEffect);
            gamepad2.runRumbleEffect(rumbleEffect);
            Gamepad.LedEffect ledEffect = new Gamepad.LedEffect.Builder()
                    .addStep(255, 255, 255, 250)  // White for 250ms
                    .build();
            gamepad1.runLedEffect(ledEffect);
            gamepad2.runLedEffect(ledEffect);
            endgameAlertTriggered = true;
        }
        if (!parkAlertTriggered && (runtime.seconds() > 110)) {
            // Create a Rumble effect that cycles through motors
            Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 500 mSec
                    .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                    .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 250 mSec
                    .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                    .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 250 mSec
                    .build();
            gamepad1.runRumbleEffect(rumbleEffect);
            gamepad2.runRumbleEffect(rumbleEffect);
            // Create a LED effect that cycles through colors
            Gamepad.LedEffect ledEffect = new Gamepad.LedEffect.Builder()
                    .addStep(255, 0, 0, 250)  // Red for 250ms
                    .addStep(0, 255, 0, 250)  // Green for 250ms
                    .addStep(0, 0, 255, 250)  // Blue for 250ms
                    .build();
            gamepad1.runLedEffect(ledEffect);
            gamepad2.runLedEffect(ledEffect);
            parkAlertTriggered = true;
        }

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.addTelemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        Pose2D drivePosition = drive.getPosition();

        telemetry.addData("Drive/Strafe", "Right Stick")
                .addData("Turn", "Left Stick + DPad for Heading Lock")
                .addData("Wrist Up/Middle/Down", "TBA")
                .addData("Claw Open/Closed", "TBA")
                .addData("\n Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", drivePosition.getX(DistanceUnit.INCH), drivePosition.getY(DistanceUnit.INCH), drivePosition.getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        CommandScheduler.getInstance().reset();
    }

    public enum Alliance {RED, BLUE}
}

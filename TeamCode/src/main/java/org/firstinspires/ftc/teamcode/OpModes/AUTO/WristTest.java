package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Hand;

@Autonomous(name = "WristActionTest", preselectTeleOp = "TeleOp")
public class WristTest extends OpMode {

    Hand hand; // claw and wrist
    VoltageSensor battery;
    ElapsedTime runtime = new ElapsedTime();

    public long SECONDS_TO_MILLISECONDS = 1000;
    public long LONG_TIMEOUT = 5 * SECONDS_TO_MILLISECONDS;
    public long SHORT_TIMEOUT = SECONDS_TO_MILLISECONDS;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        hand = new Hand(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");

        CommandScheduler.getInstance().registerSubsystem(hand);

        hand.reset();
    }

    @Override
    public void init_loop() {
        hand.readSensors();
    }

    public Command doNothing(long timeout) {
        return new CommandBase() {
        }.withTimeout(timeout);
    }

    @Override
    public void start() {
        runtime.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                hand.handTo(1, 1).withTimeout(SHORT_TIMEOUT),
                                hand.handTo(1, 0).withTimeout(SHORT_TIMEOUT)
                        )
                )
        );
    }

    @Override
    public void loop() {
        hand.readSensors();

        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket(false);
        pack.put("Elapsed Time", runtime.toString());
        pack.put("Time", time);
        pack.put("Battery", battery.getVoltage());
        hand.addTelemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        hand.readSensors();
        hand.stop();

        CommandScheduler.getInstance().reset();
    }
}

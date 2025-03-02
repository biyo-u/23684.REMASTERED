package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;


@Autonomous(name="AutoTest")
public class AutoTestNew extends OpMode {

    Drive drive;
    VoltageSensor battery;
    ElapsedTime runtime = new ElapsedTime();

    public enum Alliance {RED, BLUE};
    public AutoChamberTest.Alliance getAlliance() { return AutoChamberTest.Alliance.RED; }

    @Override
    public void init() {
        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        drive = new Drive(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);

        drive.reset();
    }

    public Command doNothing(long timeout) {
        return new CommandBase() {}.withTimeout(timeout);
    }

    @Override
    public void start() {
        // set starting position

        drive.setPosition(new Pose2D(DistanceUnit.INCH, 0.43, -1.61, AngleUnit.DEGREES, 0));
        runtime.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    drive.moveQuickly(0, 24, 0),
                    drive.moveQuickly(0, 0, 0)
                )
        );
    }

    @Override
    public void loop() {
        drive.readSensors();

        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.addTelemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        // note, seems that "drawing stuff" commands have to go in their own packet
    }

    @Override
    public void stop() {
        drive.readSensors();
        drive.stop();
    }
}

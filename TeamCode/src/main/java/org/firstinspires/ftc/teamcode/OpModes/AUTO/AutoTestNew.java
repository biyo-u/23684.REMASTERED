package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

@Autonomous(name = "AutoTest")
public class AutoTestNew extends OpMode {
    Drive drive;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(drive);
        drive.reset();
    }

    @Override
    public void init_loop() {
        drive.readSensors();
    }

    @Override
    public void start() {
        // Set start position
        drive.setPosition(new Pose2D(DistanceUnit.INCH, -32.25, -62, AngleUnit.DEGREES, 0));

        // Schedule robot movements
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                drive.moveTo(-48, -48, -135).withTimeout(5000)
                        )
                )
        );
    }

    @Override
    public void loop() {
        drive.readSensors();
        CommandScheduler.getInstance().run();
        TelemetryPacket pack = new TelemetryPacket(false);
        drive.addTelemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        drive.readSensors();
        drive.stop();

        // TODO: Persist robot position

        CommandScheduler.getInstance().reset();
    }
}

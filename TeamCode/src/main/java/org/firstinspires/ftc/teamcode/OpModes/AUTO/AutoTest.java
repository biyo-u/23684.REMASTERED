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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;


@Autonomous(name="AutoTest")
public class AutoTest extends OpMode {

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

        drive.setPosition(new Pose2D(DistanceUnit.METER, 0.43, -1.61, AngleUnit.DEGREES, 0));
        runtime.reset();

        // notes
        // robot-relative, from starting at "right edge of 3rd tile from human" (A4)
        // -> from start, we move 87cm forward, 31cm left
        // -> move back to where we were, but facing the human player

        // actuals from smashing the chamber
        //position-x-cm: -8.23974609375
        //position-y: 0.294189453125
        //position-y-cm: 29.4189453125


        // notes for "backwards specimen score"
        // shoulder_actual = 69
        // shoulder starts at ext = 0
        // shoulder score ext = 1219
        // (skunkwords still)

        // "score x" changes each time
        double score_y = -28.3;
        double pickup_x = 0.56;
        double pickup_y = -1.56;

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        // action one: move first
                        new ParallelCommandGroup(
                                drive.moveQuickly(3.1, score_y, 0).withTimeout(3000)
                        ),

                        // action two: move again and then turn
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        drive.moveQuickly(29.9, 46, 0).withTimeout(3000),
                                        drive.turnQuickly(29.9, 46, -45).withTimeout(3000)
                                )
                        )
                )
        );
    }

    @Override
    public void loop() {
        drive.read_sensors(time);

        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        // note, seems that "drawing stuff" commands have to go in their own packet
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        drive.stop();
    }
}

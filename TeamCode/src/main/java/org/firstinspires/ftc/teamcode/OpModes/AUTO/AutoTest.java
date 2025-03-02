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

// FIXME: EVERYTHING???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????

@Autonomous(name="AutoTest")
public class AutoTest extends OpMode {
    Drive drive;
    VoltageSensor battery;
    ElapsedTime runtime = new ElapsedTime();

    /**
     * @noinspection unused
     */
    public AutoTest.Alliance getAlliance() { return AutoTest.Alliance.RED; }

    @Override
    public void start() {
        drive.setPosition(new Pose2D(DistanceUnit.INCH, 0.43, -1.61, AngleUnit.DEGREES, 0));
        runtime.reset();

        double scoreY = -28.3;

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        // action one: move first
                        new ParallelCommandGroup(
                                drive.moveQuickly(3.1, scoreY).withTimeout(3000)
                        ),

                        // action two: move again and then turn
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        drive.moveQuickly(29.9, 46).withTimeout(3000),
                                        drive.turnQuickly(45).withTimeout(3000)
                                )
                        ),

                        // action three: return.
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        drive.turnQuickly(0).withTimeout(2400),
                                        drive.moveQuickly(0, -20).withTimeout(3000)
                                )
                        )
                )
        );
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        drive = new Drive(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");

        CommandScheduler.getInstance().registerSubsystem(drive);

        drive.reset();
    }

    public Command doNothing(long timeout) {
        return new CommandBase() {}.withTimeout(timeout);
    }

    /**
     * @noinspection unused
     */
    public enum Alliance {RED, BLUE}

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

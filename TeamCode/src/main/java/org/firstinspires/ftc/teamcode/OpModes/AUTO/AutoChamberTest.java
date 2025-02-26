package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

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

@Autonomous(name="AutoChamberTest")
public class AutoChamberTest extends OpMode {
    Drive drive;
//    Arm arm; TODO: create arm subsystem!!!!!
    WebcamName camera;
    VoltageSensor battery;

    ElapsedTime runtime = new ElapsedTime();

    public enum Alliance {RED, BLUE};
    public Alliance getAlliance() { return Alliance.RED; }

    // notes:
    // we are starting at the right edge of the "middle"-right tile
    // (i.e. 3 tiles from the human player zone)

    @Override
    public void init() {
        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        //arm = new Arm(hardwareMap, getAlliance() == Alliance.RED);
        drive = new Drive(hardwareMap);
        //arm.drive = drive;
        //drive.arm = arm;
        battery = hardwareMap.voltageSensor.get("Control Hub");

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);
        //CommandScheduler.getInstance().registerSubsystem(arm);

        //arm.reset();
        drive.reset();
    }

    @Override
    public void init_loop() {
        //arm.read_sensors(); TODO: create arm subsystem!!!!!
        //arm.periodic();
        // maaaaybe could do this, but ... dangerous?
        //CommandScheduler.getInstance().run();
    }

    public Command doNothing(long timeout) {
        return new CommandBase() {}.withTimeout(timeout);
    }

    // origin is center of the field, in meters
    // these measurements seem to be to "the OTOS" center
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
        double score_y = -0.72;
        double pickup_x = 0.56;
        double pickup_y = -1.56;

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        // score our preload

                        new ParallelCommandGroup(
                                //arm.highChamber(),
                                drive.moveQuickly(-0.08, score_y, 0).withTimeout(2000)
                        ),
                        //arm.highChamberScore(true),
                        //arm.highChamberScore(false),


                        // get first spike-mark sample
                        new ParallelCommandGroup(
                                drive.moveQuickly(0.76, -1.17, -45).withTimeout(2400),
                                new SequentialCommandGroup(
                                        doNothing(500)
                                        //arm.moveTo(40,200,0,0.2,Arm.CLAW_OPEN)
                                )
                        ),
                        //arm.moveTo(0,1800,0,0.2,Arm.CLAW_OPEN),
                        //arm.moveTo(0,1800,0,0.2,Arm.CLAW_CLOSED),
                        new ParallelCommandGroup(
                                //arm.moveTo(0,1800,0.5,0.45,Arm.CLAW_CLOSED),
                                drive.moveCarefully(0.77, -1.10, -135).withTimeout(1300)
                        ),
                        //drop off in human player zone
                        //arm.moveTo(0,1800,0.5,0.45,Arm.CLAW_OPEN),

                        //go back to second spike mark
                        // (the "just turn" timeouts are only 1500 because it "should' be fast)
                        new ParallelCommandGroup(
                                //arm.moveTo(0,1800,0,0.2,Arm.CLAW_OPEN),
                                drive.moveQuickly(1.03, -1.17, -45).withTimeout(1800)
                        ),
                        //grab spike mark
                        //arm.moveTo(0,1800,0,0.2,Arm.CLAW_CLOSED),
                        //drop in human zone
                        new ParallelCommandGroup(
                                //arm.moveTo(0,1800,0.5,0.45,Arm.CLAW_CLOSED),
                                drive.moveCarefully(0.77, -1.10, -135).withTimeout(1300)
                        ),
                        //arm.moveTo(0,1800,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN),
                        //arm.moveTo(0,500,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN),

                        // grab completed specimen from human #1
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        doNothing(500)
                                        //arm.moveTo(0,10,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN)
                                ),
                                drive.moveQuickly(pickup_x, pickup_y, -90).withTimeout(1800)
                        ),
                        //arm.moveTo(0, 1750, 0.15, Arm.PALM_MIDDLE, Arm.CLAW_OPEN),
                        //arm.moveTo(0, 1750, 0.15, Arm.PALM_MIDDLE, Arm.CLAW_CLOSED),

                        // score it
                        new ParallelCommandGroup(
                                //arm.highChamber(),
                                drive.moveQuickly(0.0, score_y, 0).withTimeout(1800)
                        ),
                        //arm.highChamberScore(true),
                        //arm.highChamberScore(false),

                        // grab completed speciment from human #2
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        doNothing(500)
                                        //arm.moveTo(35,10,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN),
                                        //arm.moveTo(0,10,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN)
                                ),
                                drive.moveQuickly(pickup_x, pickup_y, -90).withTimeout(2000)
                        ),
                        //arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_OPEN),
                        //arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_CLOSED),

                        // score it
                        new ParallelCommandGroup(
                                //arm.highChamber(),
                                drive.moveQuickly(0.08, score_y, 0).withTimeout(1800)
                        ),
                        //arm.highChamberScore(true),
                        //arm.highChamberScore(false),

                        // grab completed speciment from human #3
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        doNothing(500)
                                        //arm.moveTo(35,10,0.5,0.5,Arm.CLAW_OPEN),
                                        //arm.moveTo(0,10,0.5,0.5,Arm.CLAW_OPEN)
                                ),
                                drive.moveQuickly(pickup_x, pickup_y, -90).withTimeout(2000)
                        ),
                        //arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_OPEN),
                       // arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_CLOSED),

                        // score it
                        new ParallelCommandGroup(
                                //arm.highChamber(),
                                drive.moveQuickly(0.16, score_y, 0).withTimeout(1800)
                        ),
                        //arm.highChamberScore(true),
                        //arm.highChamberScore(false),

                        // try to park
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        doNothing(500)
                                        //arm.moveTo(0,1800,0.5,0.5,Arm.CLAW_OPEN)
                                ),
                                drive.moveQuickly(0.85, -1.65, -90).withTimeout(2800)
                        )
                )
        );
    }

    @Override
    public void loop() {
        drive.read_sensors(time);
        //arm.read_sensors();

        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.add_telemetry(pack);
        //arm.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        // note, seems that "drawing stuff" commands have to go in their own packet
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
        editor.putFloat("x", (float)drive.getPosition().getX(DistanceUnit.METER));
        editor.putFloat("y", (float)drive.getPosition().getY(DistanceUnit.METER));
        //editor.putFloat("extension", (float)arm.current_extension_distance);
        //editor.putFloat("shoulder_angle",(float) arm.current_shoulder_angle);
        editor.apply();
        drive.stop();
    }
}

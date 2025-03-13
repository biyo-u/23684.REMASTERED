package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Hand;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Utilites.ConstantsPro;

@Autonomous(name = "Chamber Auto", preselectTeleOp = "TeleOp")
public class ChamberAuto extends OpMode {

    public long SECONDS_TO_MILLISECONDS = 1000;
    public long LONG_TIMEOUT = 5 * SECONDS_TO_MILLISECONDS;
    public long SHORT_TIMEOUT = (long) (1.5 * SECONDS_TO_MILLISECONDS);
    Drive drive;
    Lift lift;
    Arm arm;
    Hand hand;
    VoltageSensor battery;
    ElapsedTime runtime = new ElapsedTime();
    TelemetryPacket telemetryPacket;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        hand = new Hand(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");
        telemetryPacket = new TelemetryPacket(false);

        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(lift);
        CommandScheduler.getInstance().registerSubsystem(arm);
        CommandScheduler.getInstance().registerSubsystem(hand);

        drive.reset();
        lift.reset();
        arm.reset();
        hand.reset();
    }

    public Command pause(long timeout) {
        return new CommandBase() {
        }.withTimeout(timeout);
    }

    @Override
    public void init_loop() {
        drive.readSensors();
        lift.readSensors();
        hand.readSensors();
    }

    @Override
    public void start() {
        drive.setPosition(new Pose2D(DistanceUnit.INCH, 12, -62, AngleUnit.DEGREES, 0));
        runtime.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // arms and move to prep to score preloaded specimen
                        new ParallelCommandGroup(
                                lift.liftTo(ConstantsPro.LIFT_PRESETS.CHAMBER).withTimeout(LONG_TIMEOUT),
                                arm.riseTo(ConstantsPro.SHOULDER_PRESETS.CHAMBER, telemetryPacket).withTimeout(LONG_TIMEOUT),
                                hand.handTo(1, 1).withTimeout(SHORT_TIMEOUT)
                        ),

//                        // Move to chamber and snap specimen on chamber
                        drive.moveTo(0, -42, 0).withTimeout(LONG_TIMEOUT)//,
//                        arm.riseTo(ConstantsPro.SHOULDER_PRESETS.CHAMBER, telemetryPacket).withTimeout(SHORT_TIMEOUT), // todo: ensure change when scoring
//
//                        // score on chamber as you back up to release
//                        new SequentialCommandGroup(
//                                drive.moveTo(-0, -28, 0).withTimeout(LONG_TIMEOUT), // TODO: FIND SCORING WAYPOINT (0, -y-10)
//                                hand.handTo(1, 0).withTimeout(SHORT_TIMEOUT)
//                        )//,

//                        // move to observation zone, park and prepare for teleop
//                        new SequentialCommandGroup(
//                                drive.moveTo(47, -40, 0).withTimeout(SHORT_TIMEOUT), // TODO: FIND OBSERVATION ZONE WAYPOINT (X, -Y) (more than (-51, -51)
//                                hand.handTo(0, 1).withTimeout(SHORT_TIMEOUT),
//                                lift.liftTo(ConstantsPro.LIFT_PRESETS.HOME).withTimeout(SHORT_TIMEOUT),
//                                arm.riseTo(ConstantsPro.SHOULDER_PRESETS.HOME, telemetryPacket).withTimeout(SHORT_TIMEOUT)
//                        )
                )
        );
    }

    @Override
    public void loop() {
        drive.readSensors();
        lift.readSensors();
        hand.readSensors();

        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket(false);
        pack.put("Elapsed Time", runtime.toString());
        pack.put("Time", time);
        pack.put("Battery", battery.getVoltage());
        drive.addTelemetry(pack);
        lift.addTelemetry(pack);
        arm.addTelemetry(pack);
        hand.addTelemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public void stop() {
        drive.readSensors();
        lift.readSensors();
        hand.readSensors();

        drive.stop();
        lift.stop();
        arm.stop();
        hand.stop();

        CommandScheduler.getInstance().reset();
    }
}

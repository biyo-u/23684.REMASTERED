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
        drive.setPosition(new Pose2D(DistanceUnit.INCH, 12, -62, AngleUnit.DEGREES, 0));
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
                        drive.moveTo(0, -37, 0).withTimeout(LONG_TIMEOUT),
                        new SequentialCommandGroup(
                                hand.handTo(1, 0).withTimeout(SHORT_TIMEOUT),
                                pause(SHORT_TIMEOUT),
                                hand.handTo(0, 1).withTimeout(SHORT_TIMEOUT)
                        ),
                        new SequentialCommandGroup(
                                drive.moveTo(48, -69, 270).withTimeout(SHORT_TIMEOUT)  // TODO: FIND OBSERVATION ZONE WAYPOINT (X, -Y) (more than (-51, -51)
                                // todo: add in drop lift
                                // todo: add in drop arm
                                // todo: add in wrist down + claw open
                                // todo: add in pause to wait for human player to position specimen
                        )

//                        new SequentialCommandGroup(
//                                // todo: repeat move to chamber and snap specimen on chamber code
                                 // todo: add in park
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

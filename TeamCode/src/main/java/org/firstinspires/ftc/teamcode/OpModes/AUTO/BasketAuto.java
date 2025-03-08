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
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Hand;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Utilites.ConstantsPro;

@Autonomous(name = "Basket Auto", preselectTeleOp = "TeleOp")
public class BasketAuto extends OpMode {

    public long SECONDS_TO_MILLISECONDS = 1000;
    public long LONG_TIMEOUT = 5 * SECONDS_TO_MILLISECONDS;
    public long SHORT_TIMEOUT = SECONDS_TO_MILLISECONDS;
    Drive drive; // drivetrain
    Lift lift; // viper slides / elevators
    Arm arm; // shoulder
    Hand hand; // claw and wrist
    VoltageSensor battery;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        hand = new Hand(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");

        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(lift);
        CommandScheduler.getInstance().registerSubsystem(arm);
        CommandScheduler.getInstance().registerSubsystem(hand);

        drive.reset();
        lift.reset();
        arm.reset();
        hand.reset();
    }

    @Override
    public void init_loop() {
        drive.readSensors();
        lift.readSensors();
        arm.readSensors();
        hand.readSensors();
    }

    public Command doNothing(long timeout) {
        return new CommandBase() {
        }.withTimeout(timeout);
    }

    @Override
    public void start() {
        drive.setPosition(new Pose2D(DistanceUnit.INCH, -32.25, -62, AngleUnit.DEGREES, 0));
        runtime.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // raise lift and move forward a bit
                        new ParallelCommandGroup(
                                drive.moveTo(-32.25, -60, 0).withTimeout(SHORT_TIMEOUT),
                                lift.liftTo(ConstantsPro.LIFT_PRESETS.BASKET).withTimeout(LONG_TIMEOUT),
                                arm.riseTo(ConstantsPro.SHOULDER_PRESETS.BASKET).withTimeout(LONG_TIMEOUT)
                        ),

                        // move to baskets
                        new ParallelCommandGroup(
                                drive.moveTo(-51, -51, -135).withTimeout(LONG_TIMEOUT),
                                hand.handTo(1, 1).withTimeout(SHORT_TIMEOUT)
                        ),

                        // release preload
                        new SequentialCommandGroup(
                                hand.handTo(1, 0).withTimeout(SHORT_TIMEOUT)
                        )
                )
        );
    }

    @Override
    public void loop() {
        drive.readSensors();
        lift.readSensors();
        arm.readSensors();
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
    }

    @Override
    public void stop() {
        drive.readSensors();
        lift.readSensors();
        arm.readSensors();
        hand.readSensors();

        drive.stop();
        lift.stop();
        arm.stop();
        hand.stop();

        CommandScheduler.getInstance().reset();
    }
}

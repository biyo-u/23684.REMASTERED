package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.concurrent.TimeUnit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Utilites.ConstantsPro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Hand extends SubsystemBase {
    private final SimpleServo claw;
    private final SimpleServo wrist;

    private double TelemtryWristTarget = 0;
    private double TelemtryClawTarget = 0;

    Timing.Timer clawTimer;
    Timing.Timer wristTimer;

    public Hand(HardwareMap hardwareMap) {
        this.wrist = new SimpleServo(hardwareMap, "wrist", 0, 360, AngleUnit.DEGREES);
        this.claw = new SimpleServo(hardwareMap, "claw", 0, 360, AngleUnit.DEGREES);
    }

    public void reset() {
        wrist.setPosition(0);
        claw.setPosition(1);
    }

    public Command handTo (double wristTarget, double clawTarget) {
        return new WristAndClawMove(wristTarget, clawTarget);
    }

    public void periodic() {
//        wrist.setPosition(WRIST_TARGET);
//        claw.setPosition(CLAW_TARGET);
    }

    public class WristAndClawMove extends CommandBase {

        double WRIST_TARGET = ConstantsPro.WRIST_AND_CLAW_PRESETS.WRIST_UP;
        double CLAW_TARGET = ConstantsPro.WRIST_AND_CLAW_PRESETS.CLAW_CLOSED;

        public WristAndClawMove(double wristPos, double clawPos) {
            WRIST_TARGET = wristPos;
            CLAW_TARGET = clawPos;
            TelemtryClawTarget = CLAW_TARGET;
            TelemtryWristTarget = WRIST_TARGET;

            if (wrist.getPosition() != WRIST_TARGET) {
                wristTimer = new Timing.Timer(8, TimeUnit.SECONDS);
            } else {
                wristTimer = new Timing.Timer(8, TimeUnit.SECONDS);
            }

            if (claw.getPosition() != CLAW_TARGET) {
                clawTimer = new Timing.Timer(8, TimeUnit.SECONDS); //TODO: tweak length of wait
            } else {
                clawTimer = new Timing.Timer(8, TimeUnit.SECONDS);
            }

            addRequirements(Hand.this);
        }

        @Override
        public void execute() {
            wrist.setPosition(WRIST_TARGET);
            claw.setPosition(CLAW_TARGET);
        }

        @Override
        public boolean isFinished() {
            return wristTimer.done() && clawTimer.done();
        }
    }

    public void stop() {
        wrist.setPosition(ConstantsPro.WRIST_AND_CLAW_PRESETS.WRIST_UP);
        claw.setPosition(ConstantsPro.WRIST_AND_CLAW_PRESETS.CLAW_CLOSED);
    }

    public void readSensors() {
        wrist.getPosition();
        claw.getPosition();
    }

    public void addTelemetry(TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Wrist Position", wrist.getPosition());
        telemetryPacket.put("Claw Position", claw.getPosition());
        telemetryPacket.put("Wrist Target", TelemtryWristTarget);
        telemetryPacket.put("Claw Target", TelemtryClawTarget);
        telemetryPacket.put("Time till Claw Opens", clawTimer.remainingTime());
        telemetryPacket.put("Time till Wrist Comes Down", wristTimer.remainingTime());
    }
}
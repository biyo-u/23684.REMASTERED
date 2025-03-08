package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Utilites.ConstantsPro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Hand extends SubsystemBase {
    private final SimpleServo claw;
    private final SimpleServo wrist;

    double WRIST_TARGET = ConstantsPro.WRIST_AND_CLAW_PRESETS.WRIST_UP;
    double CLAW_TARGET = ConstantsPro.WRIST_AND_CLAW_PRESETS.CLAW_CLOSED;

    public Hand(HardwareMap hardwareMap) {
        this.wrist = new SimpleServo(hardwareMap, "wrist", 0, 360, AngleUnit.DEGREES);
        this.claw = new SimpleServo(hardwareMap, "claw", 0, 360, AngleUnit.DEGREES);
    }

    public void reset() {
        wrist.setPosition(ConstantsPro.WRIST_AND_CLAW_PRESETS.WRIST_UP);
        claw.setPosition(ConstantsPro.WRIST_AND_CLAW_PRESETS.CLAW_CLOSED);
    }

    public Command handTo (double wristTarget, double clawTarget) {
        return new WristAndClawMove(wristTarget, clawTarget);
    }

    public void periodic() {
        wrist.setPosition(WRIST_TARGET);
        claw.setPosition(CLAW_TARGET);
    }

    public class WristAndClawMove extends CommandBase {
        public WristAndClawMove(double clawPos, double wristPos) {
            WRIST_TARGET = wristPos;
            CLAW_TARGET = clawPos;
            wrist.setPosition(WRIST_TARGET);
            claw.setPosition(CLAW_TARGET);
            addRequirements(Hand.this);
        }

        @Override
        public void execute() {
            claw.setPosition(CLAW_TARGET);
            wrist.setPosition(WRIST_TARGET);
        }

        @Override
        public boolean isFinished() {
            return true;
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
    }
}
package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Intake extends SubsystemBase {

    public static PIDFCoefficients shoulderPIDF = new PIDFCoefficients(0.002, 0, 0, 0);
    private final Motor shoulder;
    private final SimpleServo claw;
    private final SimpleServo wrist;
    public static double shoulderTolerance;
    private double shoulderPosition = 0;
    private double shoulderTarget = 0;
    public static double shoulderTicksPerAngle = 1;

    public double WRIST_UP = 0;
    public double WRIST_DOWN = 0;
    public double CLAW_CLOSED = 0;
    public double CLAW_OPEN = 0;
    public double shoulderPower = 0;

    public Intake(HardwareMap hardwareMap) {
        this.shoulder = new Motor(hardwareMap, "shoulder");
        this.wrist = new SimpleServo(hardwareMap, "wrist", 0, 360, AngleUnit.DEGREES);
        this.claw = new SimpleServo(hardwareMap, "claw", 0, 360, AngleUnit.DEGREES);
    }

    public void reset() {
        shoulder.stopAndResetEncoder();
    }

    public Command riseTo (double target) {
        return new RiseTo(target);
    }

    public void periodic() {
        shoulder.set(shoulderPower);
    }

    public class RiseTo extends CommandBase {

        private final PIDFController shoulderController;

        public RiseTo(double target) {
            shoulderController = new PIDFController(shoulderPIDF.p, shoulderPIDF.i, shoulderPIDF.d, shoulderPIDF.f);
            shoulderController.setTolerance(shoulderTolerance);
            shoulderTarget = target * shoulderTicksPerAngle;

            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            shoulder.set(0);
        }

        @Override
        public void execute() {
            shoulderPower = shoulderController.calculate(shoulderPosition, shoulderTarget);
        }

        @Override
        public boolean isFinished() {
            return shoulderController.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            shoulder.set(0);
        }
    }

    public class WristAndClaw extends CommandBase {
        public WristAndClaw() {
            wrist.setPosition(WRIST_UP);
            claw.setPosition(CLAW_CLOSED);
            addRequirements(Intake.this);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public void stop() {
        shoulder.set(0);
    }

    public void readSensors() {
        shoulderPosition = shoulder.getCurrentPosition();
    }

    public void addTelemetry(TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Shoulder Position", shoulderPosition);
        telemetryPacket.put("Target Shoulder", shoulderTarget);
        telemetryPacket.put("Wrist Position", wrist.getPosition());
        telemetryPacket.put("Claw Position", claw.getPosition());
    }
}
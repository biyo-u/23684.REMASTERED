package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Arm extends SubsystemBase {

    public static PIDFCoefficients shoulderPIDF = new PIDFCoefficients(0.002, 0, 0, 0);
    public static double shoulderTicksPerAngle = 1;
    private final Motor shoulder;
    public double shoulderPower = 0;

    public Arm(HardwareMap hardwareMap) {
        this.shoulder = new Motor(hardwareMap, "shoulder");
    }

    public void reset() {
        shoulder.stopAndResetEncoder();
    }

    public Command riseTo(double target, TelemetryPacket telemetryPacket) {
        return new RiseTo(target, telemetryPacket);
    }

    public void periodic() {
        shoulder.set(shoulderPower);
    }

    public void stop() {
        shoulder.set(0);
    }

    public void addTelemetry(TelemetryPacket telemetryPacket) {
    }

    public class RiseTo extends CommandBase {
        public static double shoulderTolerance;
        private final PIDFController shoulderController;
        TelemetryPacket telemetryPacket;
        private double shoulderTarget;

        public RiseTo(double target, TelemetryPacket telemetryPacket) {
            this.telemetryPacket = telemetryPacket;

            shoulderController = new PIDFController(shoulderPIDF.p, shoulderPIDF.i, shoulderPIDF.d, shoulderPIDF.f);
            shoulderController.setTolerance(shoulderTolerance);
            shoulderTarget = target * shoulderTicksPerAngle;

            if (shoulderTarget < 100) {
                shoulderTarget = 100;
            }

            addRequirements(Arm.this);
        }

        @Override
        public void initialize() {
            shoulder.set(0);
        }

        @Override
        public void execute() {
            shoulderPower = shoulderController.calculate(shoulder.getCurrentPosition(), shoulderTarget);

            telemetryPacket.put("Shoulder Target", shoulderTarget);
            telemetryPacket.put("Shoulder Position", shoulder.getCurrentPosition());
        }

        @Override
        public boolean isFinished() {
            return shoulderController.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            stop();
        }
    }
}
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
    private final Motor shoulder;
    public static double shoulderTolerance;
    private double shoulderPosition = 0;
    private double shoulderTarget = 0;
    public static double shoulderTicksPerAngle = 1;
    public double shoulderPower = 0;

    public Arm(HardwareMap hardwareMap) {
        this.shoulder = new Motor(hardwareMap, "shoulder");
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

            addRequirements(Arm.this);
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

    public void stop() {
        shoulder.set(0);
    }

    public void readSensors() {
        shoulderPosition = shoulder.getCurrentPosition();
    }

    public void addTelemetry(TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Shoulder Position", shoulderPosition);
        telemetryPacket.put("Target Shoulder", shoulderTarget);
    }
}
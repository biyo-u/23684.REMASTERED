package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Arm extends SubsystemBase {
    public static PIDFCoefficients liftPIDF = new PIDFCoefficients(0, 0, 0, 0);
    Motor liftMotorLeft;
    Motor liftMotorRight;
    private final MotorGroup lift;
    private final PIDFController liftController;
    // TODO: Find this
    private final double liftTicksPerInch = 0;
    private double liftPosition = 0;
    private double liftTarget = 0;

    public Arm(HardwareMap hardwareMap) {
        liftMotorLeft = new Motor(hardwareMap, "liftMotorLeft", Motor.GoBILDA.RPM_117);
        liftMotorLeft.setInverted(true);

        liftMotorRight = new Motor(hardwareMap, "liftMotorRight", Motor.GoBILDA.RPM_117);
        liftMotorRight.setInverted(false);

        this.lift = new MotorGroup(liftMotorLeft, liftMotorRight);
        this.liftController = new PIDFController(liftPIDF.p, liftPIDF.i, liftPIDF.d, liftPIDF.f);
        double liftTolerance = 0.1 * liftTicksPerInch;
        this.liftController.setTolerance(liftTolerance);
    }

    public void reset() {
        lift.stopAndResetEncoder();
    }

    public Command liftTo(double target) {
        return new LiftTo(target);
    }

    @Override
    public void periodic() {
        lift.set(liftController.calculate(liftPosition, liftTarget));
    }

    public void stop() {
        lift.set(0);
    }

    public class LiftTo extends CommandBase {
        public LiftTo(double target) {
            liftTarget = target * liftTicksPerInch;

            addRequirements(Arm.this);
        }

        @Override
        public void initialize() {
            lift.set(0);
        }

        @Override
        public boolean isFinished() {
            return liftController.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            lift.set(0);
        }
    }

    public void readSensors() {
        liftPosition = lift.getCurrentPosition();
    }

    public void addTelemetry(TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Lift Position", liftPosition);
    }
}

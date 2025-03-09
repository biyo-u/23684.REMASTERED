package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Lift extends SubsystemBase {
    public static PIDFCoefficients liftPIDF = new PIDFCoefficients(0.0028, 0, 0, 0);
    Motor liftMotorLeft;
    Motor liftMotorRight;
    private final MotorGroup lift;
    private final double liftTicksPerInch = 1; // TODO: CALCULATE TICKS TO INCH IN FUTURE!!!
    private double liftPosition = 0;
    private double liftTarget = 0;
    private double liftPower = 0;

    public Lift(HardwareMap hardwareMap) {
        liftMotorLeft = new Motor(hardwareMap, "liftMotorLeft", Motor.GoBILDA.RPM_117);
        liftMotorLeft.setInverted(true);
        liftMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        liftMotorRight = new Motor(hardwareMap, "liftMotorRight", Motor.GoBILDA.RPM_117);
        liftMotorRight.setInverted(false);
        liftMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        this.lift = new MotorGroup(liftMotorLeft, liftMotorRight);
    }

    public void reset() {
        lift.stopAndResetEncoder();
        liftMotorLeft.stopAndResetEncoder();
        liftMotorRight.stopAndResetEncoder();
    }

    public Command liftTo(double target) {
        return new LiftTo(target);
    }

    @Override
    public void periodic() {
        lift.set(liftPower);
    }

    public void stop() {
        lift.set(0);
    }


    public class LiftTo extends CommandBase {
        private final PIDFController liftController;
        public LiftTo(double target) {
            liftController = new PIDFController(liftPIDF.p, liftPIDF.i, liftPIDF.d, liftPIDF.f);
            double liftTolerance = 0.1 * liftTicksPerInch;
            liftController.setTolerance(liftTolerance);
            liftTarget = target * liftTicksPerInch; // CURRENTLY ACCEPTS ONLY TICKS. CALCULATE FOR TICKS TO INCH IN FUTURE!!!
            addRequirements(Lift.this);
        }

        @Override
        public void initialize() {
            lift.set(1);
        }

        @Override
        public void execute() {
            liftPower = liftController.calculate(liftPosition, liftTarget);
        }

        @Override
        public boolean isFinished() {
//            return Math.abs(liftPosition - liftTarget) < 10;
            return liftController.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            lift.set(0);
        }
    }

    public void readSensors() {
//        liftPosition = lift.getCurrentPosition();
        double leftLiftPosition = liftMotorLeft.getCurrentPosition();
        double rightLiftPosition = liftMotorRight.getCurrentPosition();
        liftPosition = (leftLiftPosition + rightLiftPosition) / 2;
    }

    public void addTelemetry(TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Lift Position", liftPosition);
        telemetryPacket.put("Right Lift Position", liftMotorRight.getCurrentPosition());
        telemetryPacket.put("Left Lift Position", liftMotorLeft.getCurrentPosition());
        telemetryPacket.put("Target Lift", liftTarget);
    }
}

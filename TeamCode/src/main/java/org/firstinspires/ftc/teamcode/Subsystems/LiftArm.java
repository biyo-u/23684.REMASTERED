package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
public class LiftArm extends SubsystemBase {
    public static PIDCoefficients slidePID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients shoulderPID = new PIDCoefficients(0, 0, 0);
    public static double shoulder_pid_max_f = 0.35;
    public static double shoulder_pid_min_f = 0.14;
    public static double extension_pid_f = 0.08;
    public static final double ticks_per_degrees = 28.0 * (1 + 46.0 / 17.0) * (1 + 46.0 / 11.0) * 51.0 / 16.0 / 360.0; // TODO: Find this
    public static double HIGH_BASKET_EXTENSION = 2650;
    public static double MAX_EXTENSION_OUT_SPEED = 1.0;
    public static double CLAW_LIMIT = 1.0;
    public static double EXTENSION_MAX = 2600;
    public static double CLAW_OPEN = 1.0;
    public static double CLAW_DETECT_PIECE_VOLTAGE = 1.22;
    public static double CLAW_CLOSED = 0.0;
    public static long ARM_MOVE_TIMEOUT = 1000;
    public double wrist_angle = 1.0;
    public static double SHOULDER_MAX_UP_POWER = 0.7;
    public double claw_position = CLAW_CLOSED;
    public AnalogInput wrist_input, palm_input, claw_input;
    public static double SHOULDER_MAX_DOWN_POWER = -0.5;
    public SimpleServo wrist_servo;
    public SimpleServo claw_servo;
    double current_shoulder_angle;
    PIDController shoulder_control;
    PIDController extension_control;
    double target_extension = 0;
    double current_extension_distance = 0.0;
    Motor liftMotorLeft;
    Motor liftMotorRight;
    Motor shoulder;
    double target_angle = 0.0;

    public LiftArm(HardwareMap hardwareMap) {
        wrist_servo = new SimpleServo(hardwareMap, "wrist", 0, 360, AngleUnit.DEGREES);
        claw_servo = new SimpleServo(hardwareMap, "claw", 0, 360, AngleUnit.DEGREES);
        liftMotorLeft = new Motor(hardwareMap, "liftMotorLeft", Motor.GoBILDA.RPM_117);
        liftMotorLeft.setInverted(true);
        liftMotorRight = new Motor(hardwareMap, "liftMotorRight", Motor.GoBILDA.RPM_117);
        liftMotorRight.setInverted(false);
        shoulder = new Motor(hardwareMap,"shoulder");

        shoulder_control = new PIDController(0.0, 0.0, 0.0);
        extension_control = new PIDController(0.0, 0.0, 0.0);
        shoulder_control.setTolerance(5);
        extension_control.setTolerance(50);
    }

    public void reset() {
        shoulder.stopAndResetEncoder();
        liftMotorLeft.stopAndResetEncoder();
        liftMotorRight.stopAndResetEncoder();
    }

    // TODO: Make commands like these:
    public Command highBasketBack(boolean closed) {
        return moveTo(
                90,
                HIGH_BASKET_EXTENSION,
                0.7,
                closed ? CLAW_CLOSED : CLAW_OPEN
        ).withTimeout(ARM_MOVE_TIMEOUT);
    }

    public Command moveTo(double target_angle, double target_extension, double wrist_angle, double claw_position) {
        return new MoveTo(target_angle, target_extension, wrist_angle, claw_position);
    }

    public class MoveTo extends CommandBase {
        double[] targets = new double[5];
        ElapsedTime runtime;

        public MoveTo(double target_angle, double target_extension, double wrist_angle, double palm_angle) {
            runtime = new ElapsedTime();
            LiftArm.this.target_angle = target_angle;
            LiftArm.this.target_extension = target_extension;
            LiftArm.this.wrist_angle = wrist_angle;

            targets[0] = target_angle;
            targets[1] = target_extension;
            targets[2] = wrist_angle;
            targets[3] = palm_angle;
            targets[4] = claw_position;

            addRequirements(LiftArm.this);
        }

        @Override
        public void initialize() {
            target_angle = targets[0];
            target_extension = targets[1];
            shoulder_control.setSetPoint(target_angle);
            extension_control.setSetPoint(target_extension);
            wrist_angle = targets[2];
            claw_position = targets[4];
            runtime.reset();
        }

        @Override
        public void execute() {
            double liftSpeed = extension_control.calculate(current_extension_distance, target_extension);
            liftMotorLeft.set(liftSpeed);
            liftMotorRight.set(liftSpeed);
        }

        @Override
        public boolean isFinished() {
            return shoulder_control.atSetPoint() && extension_control.atSetPoint() && runtime.milliseconds() > 250.0;
        }

        @Override
        public void end(boolean interrupted) {
            shoulder.stopMotor();
            liftMotorLeft.stopMotor();
            liftMotorRight.stopMotor();
        }
    }

//    public class HumanInputs extends CommandBase {
//        GamepadEx operator;
//
//        public HumanInputs(GamepadEx operator) {
//            this.operator = operator;
//            addRequirements(LiftArm.this);
//        }
//
//        @Override
//        public void execute() {
//            if (operator.wasJustPressed(GamepadKeys.Button.X) || operator.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
//                toggleClaw();
//            }
//
//            if(operator.getButton(GamepadKeys.Button.DPAD_UP)){
//                wrist_angle+=WRIST_SPEED;
//                if(wrist_angle>1.0) wrist_angle=1;
//
//            }
//            if (operator.getButton(GamepadKeys.Button.DPAD_DOWN)){
//                wrist_angle-=WRIST_SPEED;
//                if(wrist_angle<0.0)wrist_angle=0.0;
//            }
//
//            if (operator.getLeftY() > 0.2 || operator.getLeftY() < -0.2) {
//                target_angle += (operator.getLeftY() * SHOULDER_MULTIPLIER);
//            }
//
//            if (operator.getRightY() > 0.2 || operator.getRightY() < -0.2) {
//                target_extension -= (operator.getRightY() * EXTENSION_MULTIPLIER);
//            }
//
//            if (operator.wasJustPressed(GamepadKeys.Button.B)) {
//                target_angle = HIGH_CHAMBER_ANGLE;
//                target_extension = HIGH_CHAMBER_EXTENSION;
//                wrist_angle = HIGH_CHAMBER_WRIST;
//            }
//            if (operator.wasJustPressed(GamepadKeys.Button.A)) {
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                moveTo(shoulder_control.getSetPoint(), HUMAN_PICKUP_EXTENSION, HUMAN_PICKUP_WRIST, claw_position),
//                                moveTo(HUMAN_PICKUP_ANGLE, HUMAN_PICKUP_EXTENSION, HUMAN_PICKUP_WRIST, claw_position)
//                        )
//                );
//            }
//
//            // high basket, forward position
//            if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
//                target_angle = HIGH_BASKET_ANGLE;
//                target_extension = HIGH_BASKET_EXTENSION;
//                wrist_angle = HIGH_BASKET_WRIST;
//            }
//
//            if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                target_angle = HIGH_BACKWARDS_ANGLE;
//                target_extension = HIGH_BACKWARDS_EXTENSION;
//                wrist_angle = HIGH_BACKWARDS_WRIST;
//            }
//
//            // "score" the thing
//            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//                target_angle = HIGH_CHAMBER_SCORE_ANGLE;
//                target_extension = HIGH_CHAMBER_SCORE_EXTENSION;
//                wrist_angle = HIGH_CHAMBER_SCORE_WRIST;
//            }
//
//            // note: these limits also affect extension limits, even
//            // though the arm "can" go further than 90 degrees, our
//            // inspection / extension story is based around a
//            // 90-degree-max
//            if (target_angle < 0.0) target_angle = 0.0;
//            if (target_angle > 90.0) target_angle = 90.0;
//
//            // another "extension limit" thing is that the wrist
//            // cannot go "backwards" too far when the arm is close to
//            // vertical
//            if (current_shoulder_angle > 80) {
//                // if we're "close to vertical" (above 80 degrees)
//                // then we need to check the wrist
//                if (wrist_angle > 0.7)
//                    wrist_angle = 0.7;
//            }
//
//            // this "35 degrees" limit is conservative, to ensure that
//            // we're inside the extension-limit at all times (so, we
//            // want to run these checks ALWAYS, not just if a button
//            // was pressed etc)
//            if (current_shoulder_angle < 35) {
//                if (target_extension > EXTENSION_HORIZONTAL_MAX)
//                    target_extension = EXTENSION_HORIZONTAL_MAX;
//            } else {
//                if (target_extension > EXTENSION_MAX)
//                    target_extension = EXTENSION_MAX;
//            }
//
//            if (target_extension < EXTENSION_MIN) target_extension = EXTENSION_MIN;
//
//            // IMPORTANT: keep the above checks "very close" to this
//            // setSetPoint so we don't burn out motors (again!)
//            shoulder_control.setSetPoint(target_angle);
//            extension_control.setSetPoint(target_extension);
//        }
//    }

    public boolean havePiece() {
        if (claw_position <= CLAW_CLOSED) {
            return claw_input.getVoltage() < CLAW_DETECT_PIECE_VOLTAGE;
        }
        return false;
    }

    private void toggleClaw() {
        claw_position = claw_position < CLAW_OPEN ? CLAW_OPEN : CLAW_CLOSED;
    }

    @Override
    public void periodic() {
        if (claw_position < CLAW_LIMIT) claw_position = CLAW_LIMIT;
        if (claw_position > 1.0) claw_position = 1.0;
        wrist_servo.setPosition(wrist_angle);
        claw_servo.setPosition(claw_position);
        double extension_percent = current_extension_distance / EXTENSION_MAX;
        double shoulder_pid_f = shoulder_pid_min_f + ((shoulder_pid_max_f - shoulder_pid_min_f) * extension_percent);

        double pid_power = shoulder_control.calculate(current_shoulder_angle);
        double feedforward = Math.cos(Math.toRadians(current_shoulder_angle)) * shoulder_pid_f;
        double power = pid_power + feedforward;
        if (power > 0.0 && power > SHOULDER_MAX_UP_POWER) power = SHOULDER_MAX_UP_POWER;
        if (power < 0.0 && power < SHOULDER_MAX_DOWN_POWER) power = SHOULDER_MAX_DOWN_POWER;
        shoulder.set(power);
        feedforward = Math.sin(Math.toRadians(current_shoulder_angle)) * extension_pid_f;
        double ext_power = extension_control.calculate(current_extension_distance) + feedforward;
        if (ext_power > MAX_EXTENSION_OUT_SPEED) ext_power = MAX_EXTENSION_OUT_SPEED;
    }

    public void read_sensors() {
        shoulder_control.setPID(shoulderPID.p, shoulderPID.i, shoulderPID.d);
        extension_control.setPID(slidePID.p, slidePID.i, slidePID.d);
        current_shoulder_angle = shoulder.getCurrentPosition() / ticks_per_degrees;
        current_extension_distance = (double) (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition()) / 2;
    }

    public void add_telemetry(TelemetryPacket pack) {
        pack.put("claw_target", claw_servo.getPosition());
        pack.put("claw_actual", claw_input.getVoltage());
        pack.put("claw_have_piece", havePiece());
        pack.put("wrist_target", wrist_servo.getPosition());
        pack.put("wrist_actual", wrist_input.getVoltage());
        pack.put("palm_actual", palm_input.getVoltage());
        pack.put("extension_ticks", current_extension_distance);
        pack.put("extension_target", target_extension);
        pack.put("extension_power_left", liftMotorLeft.get());
        pack.put("extension_power_right", liftMotorRight.get());
        pack.put("shoulder_error", current_shoulder_angle-target_angle);
        pack.put("shoulder_actual", current_shoulder_angle);
        pack.put("shoulder_target", target_angle);  // XXX rename one to match
        pack.put("shoulder_power", shoulder.get());
    }
}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

import java.util.concurrent.TimeUnit;
@Config
public class LiftArm extends SubsystemBase {
    public static PIDCoefficients slidePID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients shoulderPID = new PIDCoefficients(0, 0, 0);
    public static double shoulder_pid_max_f = 0.35;
    /// november 15, lowered this F value so we can actually hit 0
    public static double shoulder_pid_min_f = 0.14;
    public static double extension_pid_f = 0.08;

    public static double EXTENSION_HORIZONTAL_MAX = 1800;
    public static double EXTENSION_MAX = 2600;  // absolute max extension
    public static double EXTENSION_MIN = 0;

    public static double SHOULDER_MULTIPLIER = 2.5;
    public static double EXTENSION_MULTIPLIER = 88.0;

    public static double HIGH_CHAMBER_ANGLE = 73;
    public static double HIGH_CHAMBER_EXTENSION = 950;
    public static double HIGH_CHAMBER_WRIST = 0.0;

    public static double HIGH_CHAMBER_SCORE_ANGLE = 67;
    public static double HIGH_CHAMBER_SCORE_EXTENSION = 200;
    public static double HIGH_CHAMBER_SCORE_WRIST = 0.0;

    public static double HIGH_CHAMBER_DRIVE_ANGLE = 36;
    public static double HIGH_CHAMBER_DRIVE_EXTENSION = 700;
    public static double HIGH_CHAMBER_DRIVE_WRIST = 0.8;

    public static double HUMAN_PICKUP_ANGLE = 0;
    public static double HUMAN_PICKUP_EXTENSION = 20;
    public static double HUMAN_PICKUP_WRIST = 0.5;
    public static double HIGH_BASKET_ANGLE = 80;
    public static double HIGH_BASKET_EXTENSION = 2650;
    public static double HIGH_BASKET_WRIST = 0.35;
    public static double HIGH_BACKWARDS_ANGLE = 90;
    public static double HIGH_BACKWARDS_EXTENSION = 2600;
    public static double HIGH_BACKWARDS_WRIST = 0.7;
    public static double MAX_EXTENSION_OUT_SPEED = 1.0;
    public static double MAX_SCAN_SPEED = 0.45;

    // new-claw notes:
    // 0.64 is a decent "soft" closed / grab
    // 0.625 is decent "hard" grab: could be slightly firmer?
    // 0.62 is basically "as closed as it can get"

    // with 0.625 as "closed" we get these "actual" voltages:
    // for voltages, translates to: 1.31{5-9}: closed (with piece)
    // 1.33: closed (w/o piece)
    public static double CLAW_LIMIT = 1.0;
    public static double CLAW_CLOSED = 0.0;  // jan 20 re-tuned with latest claw
    public static double CLAW_OPEN = 1.0;
    public static double CLAW_DETECT_PIECE_VOLTAGE = 1.22;
    public static double LOW_RUNG_ANGLE = 50;
    public static double LOW_RUNG_EXTENSION = 0;
    public static double LOW_RUNG_WRIST = 0.4;

    public static long ARM_MOVE_TIMEOUT = 1000; // milliseconds, default timeout

//
// measuring ticks-to-mm for the extension
// measuring _movement_ of the middle stage, so "amount of extension" is 2x that
// 2.30cm, -212ticks to start
// 16.40cm, 1315 ticks
// 108.297 ticks / cm (of "middle stage" extension)
// ...so "total extension" is: 2x (108.297/ticks) + first stage + "bit at the end"
// ...or "extension distance X2 plus collapsed length"
// approximately 3200 ticks is full range
//
// from start, we can get "back" to nearly -120 ticks
// if we push out to 2500 ticks, we're "definitely" within limits
// at 2800 ticks, we're "41 inches" with room for the rear extension of the shoulder
//



    // mounting notes november 8:
    // centered for claw server = closed, right limit = open
    // centered for wrist server = parallel to ground

    public SimpleServo wrist_servo = null;
    public double wrist_angle = 1.0;
    public static double WRIST_SPEED=0.072;

    public SimpleServo claw_servo = null;
    public double claw_position = CLAW_CLOSED;

    public AnalogInput wrist_input, palm_input, claw_input;

    // Note: we will work in DEGREES for control and inputs, although
    // the motor is of course in encoder-ticks -- but we can convert
    // encoder-ticks to angles in degrees: 415 encoder-ticks is 90
    // degrees
    // note: fixed for new 51-tooth shoulder cogs by Iskander
    public static final double ticks_per_degrees = 28.0*(1+46.0/17.0) * (1+46.0/11.0)*51.0/16.0/360.0;

    double current_shoulder_angle;  // cached in read_sensors()
    PIDController shoulder_control; // will be in DEGREES, and converted to ticks later
    public static double SHOULDER_MAX_UP_POWER = 0.7;  // prevent hopping
    public static double SHOULDER_MAX_DOWN_POWER = -0.5;  // prevent hopping
    // note we do our own feed-forward calculation; could explore ArmFeedforward from FTCLib

    public static final double extension_per_mm = 100;
    PIDController extension_control;
    double target_extension = 0;  // in ticks! should be mm
    double current_extension_distance = 0.0;

    MotorGroup ViperSlides;
    Motor shoulder;
    double target_angle = 0.0;
    public double grab_angle;

    public LiftArm(HardwareMap hardwareMap, boolean isRedAlliance) {
        wrist_servo = new SimpleServo(hardwareMap, "wrist", 0, 360, AngleUnit.DEGREES);
        claw_servo = new SimpleServo(hardwareMap, "claw", 0, 360, AngleUnit.DEGREES);

        // extension is a group of two 312 RPM goBILDA motors
        // the left lift motor in the configuration will be set as the leader for the group
        Motor liftMotorLeft = new Motor(hardwareMap, "liftMotorLeft", Motor.GoBILDA.RPM_312);
        liftMotorLeft.setInverted(true);
        Motor liftMotorRight = new Motor(hardwareMap, "liftMotorRight", Motor.GoBILDA.RPM_312);
        liftMotorRight.setInverted(false);
        ViperSlides = new MotorGroup(liftMotorLeft, liftMotorRight);

        shoulder = new Motor(hardwareMap,"shoulder", Motor.GoBILDA.NONE);

        shoulder_control = new PIDController(0.0, 0.0, 0.0);
        extension_control = new PIDController(0.0, 0.0, 0.0);
        shoulder_control.setTolerance(5);
        extension_control.setTolerance(50);
    }

    public void reset() {
        // important: robot must start with arm down and fully "in" the robot.
        shoulder.stopAndResetEncoder();
        ViperSlides.stopAndResetEncoder();
    }

    // TODO: REWRITE COMMANDS TO FIT OUR USES
    public Command highBasketBack(boolean closed) {
        return moveTo(
                90, //HIGH_BASKET_ANGLE,
                HIGH_BASKET_EXTENSION,
                0.7,
                closed ? CLAW_CLOSED : CLAW_OPEN
        ).withTimeout(ARM_MOVE_TIMEOUT);
    }

//    public Command highBasketDrop() {
//        return moveTo(80, HIGH_BASKET_EXTENSION, 0.4,
//                CLAW_OPEN
//        ).withTimeout(ARM_MOVE_TIMEOUT);
//    }
//
//    public Command highChamber() {
//        //return new HighChamber();
//        return moveTo(
//                HIGH_CHAMBER_ANGLE,
//                HIGH_CHAMBER_EXTENSION,
//                HIGH_CHAMBER_WRIST,
//                CLAW_CLOSED
//        ).withTimeout(ARM_MOVE_TIMEOUT);
//    }
//
//    public Command highChamberDriveOn(boolean closed) {
//        return moveTo(
//                HIGH_CHAMBER_DRIVE_ANGLE,
//                HIGH_CHAMBER_DRIVE_EXTENSION,
//                HIGH_CHAMBER_DRIVE_WRIST,
//                closed ? CLAW_CLOSED : CLAW_OPEN
//        ).withTimeout(ARM_MOVE_TIMEOUT);
//    }
//
//    public Command highChamberScore(boolean closed) {
//        return moveTo(
//                HIGH_CHAMBER_SCORE_ANGLE,
//                HIGH_CHAMBER_SCORE_EXTENSION,
//                HIGH_CHAMBER_SCORE_WRIST,
//                closed ? CLAW_CLOSED : CLAW_OPEN
//        ).withTimeout(ARM_MOVE_TIMEOUT);
//    }
//
//    public Command grabFromHuman() {
//        return new SequentialCommandGroup(
//                moveTo(1, 1800, 0.5, CLAW_OPEN),
//                moveTo(1, 1800, 0.0, CLAW_OPEN),
//                moveTo(1, 1800, 0.0, CLAW_CLOSED),
//                moveTo(1, 1800, 1.0, CLAW_CLOSED)
//        );
//    }

    public Command homePosition() {
        return moveTo(1, 50, 1.0, CLAW_CLOSED).withTimeout(ARM_MOVE_TIMEOUT);
    }

    public Command middlePosition() {
        return moveTo(45, 200, 1.0, CLAW_OPEN).withTimeout(ARM_MOVE_TIMEOUT);
    }

    public Command grabSpikePrepare() {
        return moveTo(1, 500, 0.5, CLAW_OPEN).withTimeout(ARM_MOVE_TIMEOUT);
    }

    public Command grabSpike() {
        // todo: redo this for our design
        return new SequentialCommandGroup(
                moveTo(1, 500, 0.0, CLAW_OPEN),
                moveTo(1, 500, 0.0, CLAW_CLOSED),
                moveTo(1, 500, 1.0, CLAW_CLOSED)
        );
    }

    public Command dropInObservationZone() {
        return new SequentialCommandGroup(
//            moveTo(1, 500, 0.5, CLAW_CLOSED),
                moveTo(1, 500, 0.5, CLAW_OPEN),
                moveTo(1, 500, 1.0, CLAW_OPEN)
        );
    }


    public Command moveTo(double target_angle, double target_extension, double wrist_angle, double claw_position) {
        return new MoveTo(target_angle, target_extension, wrist_angle, claw_position);
    }

    public class DoNothing extends CommandBase {
        @Override
        public void execute() {
        }
        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class MoveTo extends CommandBase {
        double[] targets = new double[5];
        ElapsedTime runtime;

        public MoveTo(double target_angle, double target_extension, double wrist_angle, double palm_angle) {
            runtime = new ElapsedTime();
            LiftArm.this.target_angle = target_angle;
            LiftArm.this.target_extension = target_extension;
            LiftArm.this.wrist_angle = wrist_angle;
            LiftArm.this.claw_position = claw_position;

            //FIXME TODO: use Option<double> etc instead

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
        }

        @Override
        public boolean isFinished() {
            // servos dont have encoders that we use, so we always
            // wait at least 250ms on every command
            return shoulder_control.atSetPoint() && extension_control.atSetPoint() && runtime.milliseconds() > 250.0;
        }

        @Override
        public void end(boolean interrupted) {
            // FIXME TODO: do we need to do this?
            shoulder.stopMotor();
            ViperSlides.stopMotor();
//            wrist_servo.rotateBy(0);
//            claw_servo.rotateBy(0);
        }

    }

    // special case command for climber, because we want to uncap the
    // "down" motor power so we have enough power to move the robot
    public class ClimbMove extends CommandBase {
        public ClimbMove() {
            addRequirements(LiftArm.this);
        }

        @Override
        public void initialize() {
            target_angle = 5; // we never "really" get here, but ...
            target_extension = -200;
            shoulder_control.setSetPoint(target_angle);
            extension_control.setP(0.01);
            extension_control.setSetPoint(target_extension);
            wrist_angle = 0.5;
            claw_position = CLAW_OPEN;
        }
        public void execute() {
            SHOULDER_MAX_DOWN_POWER = -1.0;
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        public void end(boolean interrupted) {
            SHOULDER_MAX_DOWN_POWER = -1.0;
            // FIXME TODO: do we need to do this?
            shoulder.stopMotor();
            ViperSlides.stopMotor();
        }
    }

    // all interaction with gamepads should go through this inner class
    public class HumanInputs extends CommandBase {
        GamepadEx operator;

        public HumanInputs(GamepadEx operator) {
            this.operator = operator;
            addRequirements(LiftArm.this);
        }

        @Override
        public void execute() {
            double right_trigger = operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double left_trigger = operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (operator.wasJustPressed(GamepadKeys.Button.X) || operator.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                toggleClaw();
            }

            if(operator.getButton(GamepadKeys.Button.DPAD_UP)){
                wrist_angle+=WRIST_SPEED;
                if(wrist_angle>1.0) wrist_angle=1;

            }
            if (operator.getButton(GamepadKeys.Button.DPAD_DOWN)){
                wrist_angle-=WRIST_SPEED;
                if(wrist_angle<0.0)wrist_angle=0.0;
            }

            if (operator.getLeftY() > 0.2 || operator.getLeftY() < -0.2) {
                // shoulder up or down
                target_angle += (operator.getLeftY() * SHOULDER_MULTIPLIER);
            }

            // NOTE: nov 8, these were "backwards" to what mahie wants
            if (operator.getRightY() > 0.2 || operator.getRightY() < -0.2) {
                target_extension -= (operator.getRightY() * EXTENSION_MULTIPLIER);
            }

            // high chamber
            if (operator.wasJustPressed(GamepadKeys.Button.B)) {
                if (false) {
                    target_angle = HIGH_CHAMBER_DRIVE_ANGLE;
                    target_extension = HIGH_CHAMBER_DRIVE_EXTENSION;
                    wrist_angle = HIGH_CHAMBER_DRIVE_WRIST;
                } else {
                    target_angle = HIGH_CHAMBER_ANGLE;
                    target_extension = HIGH_CHAMBER_EXTENSION;
                    wrist_angle = HIGH_CHAMBER_WRIST;
                }
            }

            // pick up Specimen
            // TODO: bring arm in, _then_ rotate to avoid damage etc
            if (operator.wasJustPressed(GamepadKeys.Button.A)) {
                // if we're in a "far away" position like high-basket already,
                // we want to bring the arm in FIRST and then rotate shoulder
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                moveTo(shoulder_control.getSetPoint(), HUMAN_PICKUP_EXTENSION, HUMAN_PICKUP_WRIST, claw_position),
                                moveTo(HUMAN_PICKUP_ANGLE, HUMAN_PICKUP_EXTENSION, HUMAN_PICKUP_WRIST, claw_position)
//                        moveTo(HUMAN_PICKUP_ANGLE, HUMAN_PICKUP_EXTENSION, HUMAN_PICKUP_WRIST, PALM_MIDDLE, claw_position)
                        )
                );
                //target_angle = HUMAN_PICKUP_ANGLE;
                //target_extension = HUMAN_PICKUP_EXTENSION;
                //wrist_angle = HUMAN_PICKUP_WRIST;
                //palm_angle = PALM_MIDDLE;
            }

            // high basket, forward position
            if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
                target_angle = HIGH_BASKET_ANGLE;
                target_extension = HIGH_BASKET_EXTENSION;
                wrist_angle = HIGH_BASKET_WRIST;
            }

            if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                // FIXME: make static-constants for these for FTCDashboard
                target_angle = HIGH_BACKWARDS_ANGLE;
                target_extension = HIGH_BACKWARDS_EXTENSION;
                wrist_angle = HIGH_BACKWARDS_WRIST;
            }

            // "score" the thing
            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                target_angle = HIGH_CHAMBER_SCORE_ANGLE;
                target_extension = HIGH_CHAMBER_SCORE_EXTENSION;
                wrist_angle = HIGH_CHAMBER_SCORE_WRIST;
            }

            // note: these limits also affect extension limits, even
            // though the arm "can" go further than 90 degrees, our
            // inspection / extension story is based around a
            // 90-degree-max
            if (target_angle < 0.0) target_angle = 0.0;
            if (target_angle > 90.0) target_angle = 90.0;

            // another "extension limit" thing is that the wrist
            // cannot go "backwards" too far when the arm is close to
            // vertical
            if (current_shoulder_angle > 80) {
                // if we're "close to vertical" (above 80 degrees)
                // then we need to check the wrist
                // FIXME: double check extensions
                if (wrist_angle > 0.7)
                    wrist_angle = 0.7;
            }

            // this "35 degrees" limit is conservative, to ensure that
            // we're inside the extension-limit at all times (so, we
            // want to run these checks ALWAYS, not just if a button
            // was pressed etc)
            if (current_shoulder_angle < 35) {
                if (target_extension > EXTENSION_HORIZONTAL_MAX)
                    target_extension = EXTENSION_HORIZONTAL_MAX;
            } else {
                if (target_extension > EXTENSION_MAX)
                    target_extension = EXTENSION_MAX;
            }

            if (target_extension < EXTENSION_MIN) target_extension = EXTENSION_MIN;

            // IMPORTANT: keep the above checks "very close" to this
            // setSetPoint so we don't burn out motors (again!)
            shoulder_control.setSetPoint(target_angle);
            extension_control.setSetPoint(target_extension);
        }
    }

    public boolean havePiece() {
        if (claw_position <= CLAW_CLOSED) {
            // this is a VERY tiny difference between grabbed / not grabbed
            if (claw_input.getVoltage() < CLAW_DETECT_PIECE_VOLTAGE) {
                return true;
            }
        }
        return false;
    }

    private void toggleClaw() {
        claw_position = claw_position < CLAW_OPEN ? CLAW_OPEN : CLAW_CLOSED;
/*
        if (claw == ClawPosition.OPEN){
            claw = ClawPosition.CLOSED;
        } else {
            claw = ClawPosition.OPEN;
        }
 */
    }

    Rect grab_hit_zone = new Rect(140, 200, 200, 240);

    @Override
    public void periodic() {

        // NOTE: we will burn servos out if it _ever_ sets below /
        // different from what we expect (below 0.52 or above 1.0), so
        // double-check here
        if (claw_position < CLAW_LIMIT) claw_position = CLAW_LIMIT;
        if (claw_position > 1.0) claw_position = 1.0;
        wrist_servo.setPosition(wrist_angle);
        claw_servo.setPosition(claw_position);

        // ask our shoulder controller how many DEGREES it wants to
        // move, and which direction...

        // linear-interpolate between "F" values for min / max extension
        double extension_percent = current_extension_distance / EXTENSION_MAX;
        double shoulder_pid_f = shoulder_pid_min_f + ((shoulder_pid_max_f - shoulder_pid_min_f) * extension_percent);

        double pid_power = shoulder_control.calculate(current_shoulder_angle);
        // jan 23: we want to base feedforward on the ACTUAL current angle, and not the TARGET
        double feedforward = Math.cos(Math.toRadians(current_shoulder_angle)) * shoulder_pid_f;
        double power = pid_power + feedforward;
        if (power > 0.0 && power > SHOULDER_MAX_UP_POWER) power = SHOULDER_MAX_UP_POWER;
        if (power < 0.0 && power < SHOULDER_MAX_DOWN_POWER) power = SHOULDER_MAX_DOWN_POWER;
        shoulder.set(power);

        // extension controller
        feedforward = Math.sin(Math.toRadians(current_shoulder_angle)) * extension_pid_f;
        double ext_power = extension_control.calculate(current_extension_distance) + feedforward;
        // if we spool out faster than the spring can go: problems.
        if (ext_power > MAX_EXTENSION_OUT_SPEED) ext_power = MAX_EXTENSION_OUT_SPEED;
    }

    public void read_sensors() {
        shoulder_control.setPID(shoulderPID.p, shoulderPID.i, shoulderPID.d);
        extension_control.setPID(slidePID.p, slidePID.i, slidePID.d);
        current_shoulder_angle = shoulder.getCurrentPosition() / ticks_per_degrees;
        current_extension_distance = ViperSlides.getCurrentPosition();
//        // FIXME just for tuning
//        if (false && camera_open) {
//            if (palmcam.getWhiteBalanceControl().getWhiteBalanceTemperature() != WHITE_BALANCE) {
//                palmcam.getWhiteBalanceControl().setWhiteBalanceTemperature(WHITE_BALANCE);
//            }
//            if (palmcam.getExposureControl().getExposure(TimeUnit.MILLISECONDS) != EXPOSURE_MILLI) {
//                palmcam.getExposureControl().setExposure(EXPOSURE_MILLI, TimeUnit.MILLISECONDS);
//            }
//        }
    }

    public void add_telemetry(TelemetryPacket pack) {

        //FIXME only read sensors in read_sensors
        // get the voltage of analog line of Axon+ series servos
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        pack.put("claw_target", claw_servo.getPosition());
        pack.put("claw_actual", claw_input.getVoltage());
        pack.put("claw_have_piece", havePiece());
        pack.put("wrist_target", wrist_servo.getPosition());
        pack.put("wrist_actual", wrist_input.getVoltage());
        pack.put("palm_actual", palm_input.getVoltage());
        pack.put("extension_ticks", current_extension_distance);
        pack.put("extension_target", target_extension);
        pack.put("extension_power", ViperSlides.get());
        pack.put("shoulder_error", current_shoulder_angle-target_angle);
        pack.put("shoulder_actual", current_shoulder_angle);
        pack.put("shoulder_target", target_angle);  // XXX rename one to match
        pack.put("shoulder_power", shoulder.get());
        pack.put("extension_power", ViperSlides.get());
//        pack.put("laser_average",laser_average.current_value());
//        if (vision != null) {
//            pack.put("vision_scan", scanning_for_piece);
//            pack.put("vision_lock", locked_for_grab);
//            pack.put("vision_grab", can_grab);
//            pack.put("vision_angle", grab_angle);
//            pack.put("vision_fps", palmcam.getFps());
//            pack.put("vision_fps_max", palmcam.getCurrentPipelineMaxFps());
//            if (false && camera_open) {
//                pack.put("white_balance_min", palmcam.getWhiteBalanceControl().getMinWhiteBalanceTemperature());
//                pack.put("white_balance_max", palmcam.getWhiteBalanceControl().getMaxWhiteBalanceTemperature());
//                pack.put("white_balance", palmcam.getWhiteBalanceControl().getWhiteBalanceTemperature());
//                pack.put("exposure", palmcam.getExposureControl().getExposure(TimeUnit.MILLISECONDS));
//                pack.put("gain", palmcam.getGainControl().getGain());
//            }
//        }
    }
}

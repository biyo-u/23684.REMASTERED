package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

@Config
public class hyperdroidDrive extends SubsystemBase {
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.METER;
    public static double DISTANCE_TOLERANCE = 0.005; // in DISTANCE_UNITs to target
    public static double DISTANCE_TOLERANCE_LOW = 0.05; // in DISTANCE_UNITs to target
    public static double ANGLE_TOLERANCE = 1; // in ANGLE_UNITs to target
    public static double TURN_SPEED = 3;
    public static double POWER_INPUT = 2;
    public static double DEAD_ZONE = 0.1;

    public static double TURBO_FAST_SPEED = 1.0;
    public static double TURBO_SLOW_SPEED = 0.75;

    public static double STATIC_F_SENSITIVE = 0.005; //0.001; //0.04;
    public static double STATIC_F_FORWARD = 0.20; //0.15;
    public static double STATIC_F_STRAFE = 0.25;

    // "original" ones
    //public static double STATIC_F_FORWARD = 0.08;
    //public static double STATIC_F_STRAFE = 0.08;

    public static double LINEAR_SCALAR = 1.018;
    public static double ANGULAR_SCALAR = 0.995;

    MecanumDrive drivebase;
    Robot robot;

    // inputs into the drivebase, from human or auto
    double forward; // +Fwd/-Rev
    double strafe; // +Right/-Left
    double turn; // +CW/-CCW
    double desired_heading;
    private final PIDController heading_control;
    private final PIDController strafe_control;
    private final PIDController forward_control;
    public static PIDCoefficients hPID = new PIDCoefficients(0.015,0,0.0003); //adjusted November 1
    public static PIDCoefficients PID = new PIDCoefficients(50,0,3);
    //public static PIDCoefficients careful_pid = new PIDCoefficients(1.9, 0, 0.2);
    public static PIDCoefficients careful_pid = new PIDCoefficients(2.0, 0, 0.2);
    public static PIDCoefficients quick_pid = new PIDCoefficients(2.0, 0, 0.2);
    public static Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;

    Pose2D current_position;

    public hyperdroidDrive(HardwareMap hardwareMap) {
        // TO DO: replace Motor.GoBILDA.RPM_312 with CPR, RPM:
        Motor motor_fl = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        motor_fl.setInverted(true);
        motor_fl.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_fr = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        motor_fr.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_bl = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
        motor_bl.setInverted(true);
        motor_bl.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_br = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        motor_br.setZeroPowerBehavior(zeroPowerBehavior);

        drivebase = new MecanumDrive(false, motor_fl, motor_fr, motor_bl, motor_br);
        drivebase.setMaxSpeed(1);
        // FIXME: move into the MoveTo etc classes
        strafe_control  = new PIDController(PID.p,PID.i,PID.d);
        strafe_control.setTolerance(DISTANCE_TOLERANCE, Double.POSITIVE_INFINITY);
        forward_control = new PIDController(PID.p,PID.i,PID.d);
        forward_control.setTolerance(DISTANCE_TOLERANCE, Double.POSITIVE_INFINITY);

        heading_control = new PIDController(hPID.p,hPID.i,hPID.d);
        heading_control.setTolerance(ANGLE_TOLERANCE, Double.POSITIVE_INFINITY);

        // notes:
        // (above offset is the offset from the exact _center_ of the robot)
        // from CAD, December 6:
        //   - OTOS is 130.37mm from front of robot
        //   - OTOS is 154.35mm from side of robot (it's centered, so from either side)
        //   - OTOS offset is 46.635mm from exact center
    }

    public void stop() {
        drivebase.stop();
    }

    public static double wrapAngle(double angle) {
        angle %= 360; // normalize angle between -360 and +360
        if (angle > 180)
            angle -= 360;
        else if (angle <= -180)
            angle += 360;
        return angle;
    }

    public void setPosition(Pose2D pose) {
        robot.odometry.setPosition(pose);
        current_position = pose;
//        desired_heading = pose.h;
    }

    public Pose2D getPosition() {
        return current_position;
    }

    public Pose2D getTarget() {
        Pose2D findTarget = new Pose2D(DistanceUnit.INCH, strafe_control.getSetPoint(), forward_control.getSetPoint(), AngleUnit.DEGREES, desired_heading);
        return findTarget;
    }

    public Command moveTo(double x, double y, double h) {
        return moveTo(x, y, h, 0.5, DISTANCE_TOLERANCE);
    }

    public Command moveTo(double x, double y, double h, double max_speed, double distance_tolerance) {
        //return new MoveTo(x, y, h, max_speed, distance_tolerance);
        return new CarefulMoveTo(x, y, h);
    }

    public Command moveCarefully(double x, double y, double heading) {
        return new CarefulMoveTo(x, y, heading);
    }

    public Command moveQuickly(double x, double y, double heading) {
        return new QuickMoveTo(x, y, heading);
    }



    public class QuickMoveTo extends CommandBase {
        Pose2D target;
        private PIDController quick_strafe;
        private PIDController quick_forward;

        public QuickMoveTo(double x, double y, double h) {
            target = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
            quick_strafe = new PIDController(quick_pid.p, quick_pid.i, quick_pid.d);
            quick_forward = new PIDController(quick_pid.p, quick_pid.i, quick_pid.d);
            quick_strafe.setTolerance(DISTANCE_TOLERANCE_LOW);
            quick_forward.setTolerance(DISTANCE_TOLERANCE_LOW);
            addRequirements(hyperdroidDrive.this);
        }

        @Override
        public void initialize() {
            quick_strafe.setSetPoint(target.getX(DistanceUnit.INCH));
            quick_forward.setSetPoint(target.getY(DistanceUnit.INCH));
            heading_control.setSetPoint(wrapAngle(target.getHeading(AngleUnit.DEGREES)));

            // careful, take out for production FIXME TODO
            //otos.setLinearScalar(LINEAR_SCALAR);
            //robot.odometry.se
            robot.odometry.setYawScalar(ANGULAR_SCALAR);
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
        }

        @Override
        public void execute() {
            // compute the direction vector relatively to the robot coordinates
            strafe = quick_strafe.calculate(current_position.getX(DistanceUnit.INCH));
            forward = quick_forward.calculate(current_position.getY(DistanceUnit.INCH));

            if (strafe > STATIC_F_SENSITIVE) strafe += STATIC_F_STRAFE;
            if (strafe < -STATIC_F_SENSITIVE) strafe -= STATIC_F_STRAFE;
            if (forward > STATIC_F_SENSITIVE) forward += STATIC_F_FORWARD;
            if (forward < -STATIC_F_SENSITIVE) forward -= STATIC_F_FORWARD;
        }

        @Override
        public boolean isFinished() {
            // check if the target is reached
            return quick_strafe.atSetPoint() && quick_forward.atSetPoint() && heading_control.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            strafe = 0;
            forward = 0;
            stop();
        }
    }


    public class CarefulMoveTo extends CommandBase {
        Pose2D target;
        private PIDController careful_strafe;
        private PIDController careful_forward;

        public CarefulMoveTo(double x, double y, double h) {
            target = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
            careful_strafe = new PIDController(careful_pid.p, careful_pid.i, careful_pid.d);
            careful_forward = new PIDController(careful_pid.p, careful_pid.i, careful_pid.d);
            careful_strafe.setTolerance(DISTANCE_TOLERANCE);
            careful_forward.setTolerance(DISTANCE_TOLERANCE);
            addRequirements(hyperdroidDrive.this);
        }

        @Override
        public void initialize() {
            careful_strafe.setSetPoint(target.getX(DistanceUnit.INCH));
            careful_forward.setSetPoint(target.getY(DistanceUnit.INCH));
            heading_control.setSetPoint(wrapAngle(target.getHeading(AngleUnit.DEGREES)));

            // careful, take out for production FIXME TODO
            //otos.setLinearScalar(LINEAR_SCALAR);
            robot.odometry.setYawScalar(ANGULAR_SCALAR);
            drivebase.setMaxSpeed(TURBO_SLOW_SPEED);
        }

        @Override
        public void execute() {
            // compute the direction vector relatively to the robot coordinates
            strafe = careful_strafe.calculate(current_position.getX(DistanceUnit.INCH));
            forward = careful_forward.calculate(current_position.getY(DistanceUnit.INCH));

            if (strafe > STATIC_F_SENSITIVE) strafe += STATIC_F_STRAFE;
            if (strafe < -STATIC_F_SENSITIVE) strafe -= STATIC_F_STRAFE;
            if (forward > STATIC_F_SENSITIVE) forward += STATIC_F_FORWARD;
            if (forward < -STATIC_F_SENSITIVE) forward -= STATIC_F_FORWARD;
        }

        @Override
        public boolean isFinished() {
            // check if the target is reached
            return careful_strafe.atSetPoint() && careful_forward.atSetPoint() && heading_control.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            strafe = 0;
            forward = 0;
            stop();
        }
    }





    public class MoveTo extends CommandBase {
        Pose2D target;
        double max_speed;
        double distance_tolerance;

        // FIXME move the PID controllers into this class (so e.g. we
        // have have a "MoveToFast" and a "MoveToSlow" with
        // differently-tuned PIDs and different max speeds...

        public MoveTo(double x, double y, double h, double max_speed, double distance_tolerance) {
            target = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
            this.max_speed = max_speed;
            this.distance_tolerance = distance_tolerance;
            addRequirements(hyperdroidDrive.this);
        }

        public void setTarget(double x, double y, double h) {
            strafe_control.setSetPoint(x);
            forward_control.setSetPoint(y);
            desired_heading = wrapAngle(h);
        }

        @Override
        public void initialize() {
            setTarget(target.getX(DistanceUnit.INCH), target.getY(DistanceUnit.INCH), target.getHeading(AngleUnit.DEGREES));
            strafe_control.setTolerance(distance_tolerance, Double.POSITIVE_INFINITY);
            forward_control.setTolerance(distance_tolerance, Double.POSITIVE_INFINITY);
            drivebase.setMaxSpeed(max_speed);
        }

        @Override
        public void execute() {
            // compute the direction vector relatively to the robot coordinates

            // TODO: double check that re-setting these PID values so
            // frequently doesn't screw up the I or D parts
            strafe_control.setPID(PID.p, PID.i, PID.d);
            forward_control.setPID(PID.p, PID.i, PID.d);
            strafe = strafe_control.calculate(current_position.getX(DistanceUnit.INCH));
            forward = forward_control.calculate(current_position.getY(DistanceUnit.INCH));
        }

        @Override
        public boolean isFinished() {
            // check if the target is reached
            return strafe_control.atSetPoint() && forward_control.atSetPoint() && heading_control.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            strafe = 0;
            forward = 0;
            stop();
        }
    }

    // all interaction with gamepads should go through this inner class
    public class HumanInputs extends CommandBase {
        GamepadEx driver;

        public HumanInputs(GamepadEx driver) {
            this.driver = driver;
            addRequirements(hyperdroidDrive.this);
        }

        @Override
        public void execute() {
            // Run wheels in POV mode: use the Right stick to go forward & strafe, the Left stick to rotate left & right.
            strafe = scaleInputs(driver.getRightX());
            forward = scaleInputs(-driver.getRightY());
            double leftX = driver.getLeftX();
            if (Math.abs(leftX) > DEAD_ZONE)
                desired_heading = wrapAngle(desired_heading - TURN_SPEED * leftX);
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                desired_heading = 0;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                desired_heading = 180;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                desired_heading = 90;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                desired_heading = -90;

            // Anjalika wants "turbo" mode ... so if we're holding
            // left trigger _currently_, we go to Turbo -- otherwise
            // to non-Turbo
            if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
                turbo(true);
            } else {
                turbo(false);
            }
        }

        public double scaleInputs(double input) {
            if (Math.abs(input) > DEAD_ZONE)
                return Math.pow(Math.abs(input), POWER_INPUT) * Math.signum(input);
            else
                return 0;
        }
    }

    public void turbo(boolean on) {
        if (on) {
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
            // TODO might want differently-tuned heading-lock PIDs for turbo vs not-turbo
        } else {
            drivebase.setMaxSpeed(TURBO_SLOW_SPEED);
        }
    }

    // called ONCE, before any driver or robot inputs
    public void read_sensors() {
        // Get the latest pose, which includes the x and y coordinates, plus the heading angle
        current_position = robot.odometry.getPosition();
    }

    @Override
    public void periodic() {
        // heading lock
        heading_control.setPID(hPID.p,hPID.i,hPID.d);
        turn = heading_control.calculate(wrapAngle(desired_heading - current_position.getHeading(AngleUnit.DEGREES)));
        // tell ftclib its inputs
        drivebase.driveFieldCentric(strafe, forward, turn, current_position.getHeading(AngleUnit.DEGREES), false);
    }

    public void add_telemetry(TelemetryPacket pack) {
        pack.put("position-x-mm", current_position.getX(DistanceUnit.MM));
        pack.put("position-y-mm", current_position.getY(DistanceUnit.MM));
        pack.put("position-x-in", current_position.getX(DistanceUnit.INCH) * 100.0);
        pack.put("position-y-in", current_position.getY(DistanceUnit.INCH) * 100.0);
        pack.put("target-x", strafe_control.getSetPoint());
        pack.put("target-y", forward_control.getSetPoint());
        pack.put("current-heading",current_position.getHeading(AngleUnit.DEGREES));
        pack.put("desired-heading", desired_heading);
        pack.put("autonomous-heading", heading_control.getSetPoint());
    }
}

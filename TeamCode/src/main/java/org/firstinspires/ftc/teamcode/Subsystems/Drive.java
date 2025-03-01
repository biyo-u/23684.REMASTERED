package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;

@Config
public class Drive extends SubsystemBase {
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static double DISTANCE_TOLERANCE = 0.005; // in DISTANCE_UNITs to target
    public static double DISTANCE_TOLERANCE_LOW = 0.010; // in DISTANCE_UNITs to target
    public static double ANGLE_TOLERANCE = 1; // in ANGLE_UNITs to target
    public static double TURN_SPEED = 3;
    public static double POWER_INPUT = 2;
    public static double DEAD_ZONE = 0.1;

    public static double TURBO_FAST_SPEED = 1.0;
    public static double TURBO_SLOW_SPEED = 0.75;

    public static double STATIC_F_SENSITIVE = 0.005; //0.001; //0.04;
    // jan 15, 2025: re-tuned these with the higher "careful_pid"
    // jan 20, re-tuned these with "StaticFriction" op-mode
    public static double STATIC_F_FORWARD = 0.04; // 0.09;
    public static double STATIC_F_STRAFE = 0.08; // 0.15;

    public static double LINEAR_SCALAR = 1.018;
    public static double ANGULAR_SCALAR = 0.995;

    MecanumDrive drivebase;
    GoBildaPinpointDriver pinpoint;

    //double current_left_distance = 0.0; // in millimeters
    //Rev2mDistanceSensor dist_left;
    //MovingAverage dist_left_avg;  // filtered, in millimeters
    //double current_right_distance = 0.0; // in millimeters
    //Rev2mDistanceSensor dist_right;
    //MovingAverage dist_right_avg;  // filtered, in millimeters

    // inputs into the drivebase, from human or auto
    double forward; // +Fwd/-Rev
    double strafe; // +Right/-Left
    double turn; // +CW/-CCW
    double desired_heading;
    double human_stick;

    double ff_forward;
    double ff_strafe;

    private final PIDController heading_control;
    public static PIDCoefficients hPID = new PIDCoefficients(0.04,0,0); //adjusted November 1
//    public static PIDCoefficients PID = new PIDCoefficients(50,0,3);
    //public static PIDCoefficients careful_pid = new PIDCoefficients(1.9, 0, 0.2);
    //public static PIDCoefficients careful_pid = new PIDCoefficients(2.0, 0, 0.2);
    // jan 15, 2025 re-tuned this, also new static-f values
    // jan 20, 2025 -- re-tuning with separate forward/strafe PIDs (at 75% and 100% drivebase)
    static PIDCoefficients forward_pid_careful = new PIDCoefficients(0, 0, 0);
    static PIDCoefficients strafe_pid_careful = new PIDCoefficients(0, 0, 0);

    // jan 20 -- re-tuned at 100% speed
    // jan 23, 2025 -- re-tuning at max-speed
    public static PIDFCoefficients forward_pid_quick = new PIDFCoefficients(0.04, 0, 0.005, 0.0045);
    public static PIDFCoefficients strafe_pid_quick = new PIDFCoefficients(0.055, 0, 0, 0);

    public static Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;

    double current_time;
    Pose2D current_position;
    Pose2D previous_position;

    double previous_time;
    double headingWrapMultiplier = 0;

    double x_velocity;
    double y_velocity;

    Pose2D target = new Pose2D(DISTANCE_UNIT,0,0,ANGLE_UNIT,0);

    // robot geometry
    // static final double WHEEL_RADIUS = 0.048; // in DISTANCE_UNITs
    // static final double LX = 0.118;  // lateral distance from robot's COM to wheel [m].
    // static final double LY = 0.127;  // longitudinal distance from robot's COM to wheel [m].

    // static final double GEAR_RATIO = (1+46.0/17) * (1+46.0/11) * (22.0/24);
    // static final double CPR = 28*GEAR_RATIO;
    // static final double RPM = 6000/GEAR_RATIO;
    // static double MAX_SPEED = 2*WHEEL_RADIUS*Math.PI*RPM/60; // maximum chassis speed in DISTANCE_UNITs per second

// october 13, doing some OTOS calibration tests
//
// spin 10x on Adrian's "at home" tiles (not identical to production)
// -> start square to something
// -> use controller to spin 10x
// -> manually square up at the end
//
// test #1: end angle: 17.699  (spin ccw)
// test #2: end angle: -18.62  (spin cw)
// test #3: end angle: -16.9299 (spin cw)
// test #4: end angle: 17.869  (spin ccw)

//// (17.869 + 16.9299 + 18.62 + 17.699) / 4.0 = 17.779


// linear tests (1m push in one direction)
// test #1: manual push backwards: y= -1.012
// test #2: manual push backwards: y= -0.9818
// test #3: manual push backwards: y= -0.9872
// test #4: manual push backwards: y= -0.9857
// test #5: manual push backwards: y= -0.9686
// test #6: manual push backwards: y= -0.9561

    /// (-1.012 + -0.9818 + -0.9872 + -0.9857 + -0.9686 + -0.9561) / 6.0 = -0.9819



// NOTE: static-friction seems to be about "0.1" motor-power to overcome
// NOTE: we give "Drive" access to "Arm" here so that our Driver can initiate climbing
    //public Arm arm = null; TODO: CREATE ARM!!!!!!!!!!!!!

    public Drive(HardwareMap hardwareMap) {
        // TO DO: replace Motor.GoBILDA.RPM_312 with CPR, RPM:
        Motor motor_fl = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        motor_fl.setInverted(false);
        motor_fl.setZeroPowerBehavior(zeroPowerBehavior);

        Motor motor_fr = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        motor_fr.setInverted(true);
        motor_fr.setZeroPowerBehavior(zeroPowerBehavior);

        Motor motor_bl = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
        motor_bl.setInverted(false);
        motor_bl.setZeroPowerBehavior(zeroPowerBehavior);

        Motor motor_br = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        motor_br.setInverted(true);
        motor_br.setZeroPowerBehavior(zeroPowerBehavior);

        drivebase = new MecanumDrive(false, motor_fl, motor_fr, motor_bl, motor_br);
        drivebase.setMaxSpeed(1);

        heading_control = new PIDController(hPID.p,hPID.i,hPID.d);
        heading_control.setTolerance(ANGLE_TOLERANCE, Double.POSITIVE_INFINITY);

        // configure odometry sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        otos.setLinearUnit(DISTANCE_UNIT);
//        otos.setAngularUnit(ANGLE_UNIT);
//        // 169.91mm from back of arm to "center of robot"
//        // theory: our "center of drivebase" is not actually where the
//        // robot rotates around .. and so this offset isn't actually
//        // correct, causing a bit more drive when we "turn and drive"
//        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0.0466, 0));
//        // notes:
//        // (above offset is the offset from the exact _center_ of the robot)
//        // from CAD, December 6:
//        //   - OTOS is 130.37mm from front of robot
//        //   - OTOS is 154.35mm from side of robot (it's centered, so from either side)
//        //   - OTOS offset is 46.635mm from exact center
//
//        otos.setLinearScalar(LINEAR_SCALAR);
//        otos.setAngularScalar(ANGULAR_SCALAR);
//        otos.calibrateImu();

        this.pinpoint.setOffsets(-173.0, 156); //measured in mm
        this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.pinpoint.resetPosAndIMU();

        // distance sensor
        // TODO: more efficient if plugged to control-hub (not expansion)?
        /*
        dist_left = hardwareMap.get(Rev2mDistanceSensor.class, "dist_left");
        dist_left_avg = new MovingAverage(20);
        dist_right = hardwareMap.get(Rev2mDistanceSensor.class, "dist_right");
        dist_right_avg = new MovingAverage(20);
         */
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
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

    public double unnormalizeHeading(double angle) {
        if (previous_position == null) {
            previous_position = current_position;
        }

        if (previous_position.getHeading(AngleUnit.DEGREES) > 90 &&
                previous_position.getHeading(AngleUnit.DEGREES) < 181 &&
                current_position.getHeading(AngleUnit.DEGREES) < -90 && current_position.getHeading(AngleUnit.DEGREES) > -181){
            // Flipped from 180 to -180
            headingWrapMultiplier += 1;
        } else if (previous_position.getHeading(AngleUnit.DEGREES) < -90 &&
                previous_position.getHeading(AngleUnit.DEGREES) > -181 &&
                current_position.getHeading(AngleUnit.DEGREES) > 90 && current_position.getHeading(AngleUnit.DEGREES) < 181){
            // Flipped from -180 to 180
            headingWrapMultiplier -= 1;
        }

        previous_position = current_position;

        return angle + (headingWrapMultiplier * 360);
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
//        previous_position = current_position;
        current_position = pose;
        desired_heading = pose.getHeading(ANGLE_UNIT);
    }

    public Pose2D getPosition() {
        return current_position;
    }

//    public Command moveCarefully(double x, double y, double heading) {
//        return new CarefulMoveTo(x, y, heading);
//    }

    public Command moveQuickly(double x, double y, double heading) {
        return new QuickMoveTo(x, y, heading);
    }

    public class QuickMoveTo extends CommandBase {
        private PIDFController quick_strafe;
        private PIDFController quick_forward;

        public QuickMoveTo(double x, double y, double h) {
            target = new Pose2D(DISTANCE_UNIT,x, y, ANGLE_UNIT, h);
            quick_strafe = new PIDFController(strafe_pid_quick.p, strafe_pid_quick.i, strafe_pid_quick.d, strafe_pid_quick.f);
            quick_forward = new PIDFController(forward_pid_quick.p, forward_pid_quick.i, forward_pid_quick.d, forward_pid_quick.f);
            quick_strafe.setTolerance(DISTANCE_TOLERANCE_LOW);
            quick_forward.setTolerance(DISTANCE_TOLERANCE_LOW);
            addRequirements(Drive.this);
        }

        @Override
        public void initialize() {
            quick_strafe.setSetPoint(target.getX(DISTANCE_UNIT));
            quick_forward.setSetPoint(target.getY(DISTANCE_UNIT));
            desired_heading = wrapAngle(target.getHeading(ANGLE_UNIT));

//            // careful, take out for production FIXME TODO
//            if (false) {
//                otos.setLinearScalar(LINEAR_SCALAR);
//                otos.setAngularScalar(ANGULAR_SCALAR);
//            }
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
        }

        @Override
        public void execute() {
            // compute the direction vector relatively to the robot coordinates
            strafe = quick_strafe.calculate(current_position.getX(DISTANCE_UNIT));
            forward = quick_forward.calculate(current_position.getY(DISTANCE_UNIT));

            // our own "static friction" calc TODO: TUNE STATIC_F_SENSITIVE
            if (strafe > STATIC_F_SENSITIVE) ff_strafe = STATIC_F_STRAFE;
            if (strafe < -STATIC_F_SENSITIVE) ff_strafe = -STATIC_F_STRAFE;
            if (forward > STATIC_F_SENSITIVE) ff_forward = STATIC_F_FORWARD;
            if (forward < -STATIC_F_SENSITIVE) ff_forward = -STATIC_F_FORWARD;

            strafe += ff_strafe;
            forward += ff_forward;
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


//    public class CarefulMoveTo extends CommandBase {
//        Pose2D target;
//        private PIDController careful_strafe;
//        private PIDController careful_forward;
//
//        public CarefulMoveTo(double x, double y, double h) {
//            //System.out.println("x="+(x-current_position.x)+" y="+(y-current_position.y)+" h="+(h-current_position.h));
//            target = new Pose2D(DISTANCE_UNIT,x, y, ANGLE_UNIT, h);
//            careful_strafe = new PIDController(strafe_pid_careful.p, strafe_pid_careful.i, strafe_pid_careful.d);
//            careful_forward = new PIDController(forward_pid_careful.p, forward_pid_careful.i, forward_pid_careful.d);
//            careful_strafe.setTolerance(DISTANCE_TOLERANCE);
//            careful_forward.setTolerance(DISTANCE_TOLERANCE);
//            addRequirements(Drive.this);
//        }
//
//        @Override
//        public void initialize() {
//            careful_strafe.setSetPoint(target.getX(DISTANCE_UNIT));
//            careful_forward.setSetPoint(target.getY(DISTANCE_UNIT));
//            desired_heading = wrapAngle(target.getHeading(ANGLE_UNIT));
//
//            // careful, take out for production FIXME TODO
////            if (false) {
////                otos.setLinearScalar(LINEAR_SCALAR);
////                otos.setAngularScalar(ANGULAR_SCALAR);
////            }
//            drivebase.setMaxSpeed(TURBO_SLOW_SPEED);
//        }
//
//        @Override
//        public void execute() {
//            // compute the direction vector relatively to the robot coordinates
//            strafe = careful_strafe.calculate(current_position.getX(DISTANCE_UNIT));
//            forward = careful_forward.calculate(current_position.getY(DISTANCE_UNIT));
//
//            // our own "static friction" calc
//            if (strafe > STATIC_F_SENSITIVE) ff_strafe = STATIC_F_STRAFE;
//            if (strafe < -STATIC_F_SENSITIVE) ff_strafe = -STATIC_F_STRAFE;
//            if (forward > STATIC_F_SENSITIVE) ff_forward = STATIC_F_FORWARD;
//            if (forward < -STATIC_F_SENSITIVE) ff_forward = -STATIC_F_FORWARD;
//
//            strafe += ff_strafe;
//            forward += ff_forward;
//        }
//
//        @Override
//        public boolean isFinished() {
//            // check if the target is reached
//            return careful_strafe.atSetPoint() && careful_forward.atSetPoint() && heading_control.atSetPoint();
//        }
//
//        @Override
//        public void end(boolean interrupted) {
//            strafe = 0;
//            forward = 0;
//            stop();
//        }
//    }

    // all interaction with gamepads should go through this inner class
    public class HumanInputs extends CommandBase {
        GamepadEx driver;
        boolean did_prime = false;

        public HumanInputs(GamepadEx driver) {
            this.driver = driver;
            addRequirements(Drive.this);
        }

        @Override
        public void execute() {
            // Run wheels in POV mode: use the Right stick to go forward & strafe, the Left stick to rotate left & right.
            strafe = scaleInputs(driver.getRightX());
            forward = scaleInputs(-driver.getRightY());
//            double leftX = driver.getLeftX();
//            if (Math.abs(leftX) > DEAD_ZONE)
//                desired_heading = wrapAngle(desired_heading - TURN_SPEED * leftX);
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
//            if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
//                turbo(true);
//            } else {
//                turbo(false);
//            }
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
    public void read_sensors(double time) {
        // Get the latest pose, which includes the x and y coordinates, plus the heading angle
        previous_time = current_time;
        current_time = time;
        //previous_position = current_position;
        current_position = pinpoint.getPosition();
        /*
        current_left_distance= dist_left.getDistance(DistanceUnit.INCH);
        dist_left_avg.add_sample(current_left_distance);
        current_right_distance= dist_right.getDistance(DistanceUnit.INCH);
        dist_right_avg.add_sample(current_right_distance);
        */
        // if we have at least two positions, we can compute our velocity
        /*
        if (previous_position != null && (current_time - previous_time) > 0.0) {
            double interval = current_time - previous_time;
            x_velocity = (current_position.x - previous_position.x) / interval;
            y_velocity = (current_position.y - previous_position.y) / interval;
        }
        */
    }

    @Override
    public void periodic() {
        pinpoint.update();
        // heading lock
        //heading_control.setPID(hPID.p,hPID.i,hPID.d);
        heading_control.setPID(hPID.p,hPID.i,hPID.d);
        turn = heading_control.calculate(unnormalizeHeading(current_position.getHeading(ANGLE_UNIT)), desired_heading);
        // tell ftclib its inputs
        drivebase.driveFieldCentric(strafe, forward, turn, current_position.getHeading(ANGLE_UNIT), false);
    }

    public void add_telemetry(TelemetryPacket pack) {
        pack.put("x", current_position.getX(DISTANCE_UNIT));
        pack.put("y", current_position.getY(DISTANCE_UNIT));
//        pack.put("position-x (cm)", current_position.getY(DistanceUnit.CM));
//        pack.put("position-y (cm)", current_position.getY(DistanceUnit.CM));
        pack.put("current-heading-unnormalized", unnormalizeHeading(current_position.getHeading(ANGLE_UNIT)));
        pack.put("target-x", target.getX(DISTANCE_UNIT));
        pack.put("target-y", target.getY(DISTANCE_UNIT));
        pack.put("current-heading",current_position.getHeading(ANGLE_UNIT));
        pack.put("desired-heading", desired_heading);

        pack.put("strafe", strafe);
        pack.put("forward", forward);
        pack.put("strafe_ff", ff_strafe);
        pack.put("forward_ff", ff_forward);
        pack.put("turn", turn);
        /*
        pack.put("dist_left_current", current_left_distance);
        pack.put("dist_left_avg", dist_left_avg.current_value());
        pack.put("dist_right_current", current_right_distance);
        pack.put("dist_right_avg", dist_right_avg.current_value());
        */
    }
}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilites.GoBildaPinpointDriver;

@Config
public class Drive extends SubsystemBase {
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static PIDFCoefficients HEADING_PID_QUICK = new PIDFCoefficients(0.04, 0, 0.0022, 0.0005);
    public static PIDFCoefficients FORWARD_PID_QUICK = new PIDFCoefficients(0.042, 0, 0.0053, 0.0041);
    public static PIDFCoefficients STRAFE_PID_QUICK = new PIDFCoefficients(0.051, 0, 0, 0.006);
    public static double DISTANCE_TOLERANCE_LOW = 0.01;
    public static double ANGLE_TOLERANCE = 1;
    public static double ANGLE_VELOCITY_TOLERANCE = Double.POSITIVE_INFINITY;
    public static double DEAD_ZONE = 0.1;
    public static double TURN_SPEED = 3;
    public static double TURBO_FAST_SPEED = 1.0;
    public static double TURBO_SLOW_SPEED = 0.75;
    public static double STATIC_F_SENSITIVE = 0.005;
    public static Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
    MecanumDrive drivebase;
    GoBildaPinpointDriver pinpoint;
    double forward;
    double strafe;
    double turn;
    double ffForward;
    double ffStrafe;
    double ffTurn;
    Pose2D currentPosition;
    Pose2D previousPosition;
    double headingWrapMultiplier = 0;
    Pose2D target = new Pose2D(DISTANCE_UNIT, 0, 0, ANGLE_UNIT, 0);

    // TODO: CREATE ARM (READ BELOW)
    // We may not need an arm here, here's why:
    // In the Hyperdroid's code, they had their arm in the Drive class for climbing
    // Since we don't climb with our arm, we could leave it for a separate class
    // And then use it in TeleOp and our Autonomous classes.
    // Here is the original comment from Hyperdroid:
    // NOTE: we give "Drive" access to "Arm" here so that our Driver can initiate climbing

    public Drive(HardwareMap hardwareMap) {
        Motor motorFL = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        motorFL.setZeroPowerBehavior(zeroPowerBehavior);

        Motor motorFR = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        motorFR.setZeroPowerBehavior(zeroPowerBehavior);

        Motor motorBL = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
        motorBL.setZeroPowerBehavior(zeroPowerBehavior);

        Motor motorBR = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        motorBR.setZeroPowerBehavior(zeroPowerBehavior);

        drivebase = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);
        drivebase.setMaxSpeed(1);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        this.pinpoint.setOffsets(-173.0, 156);
        this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        reset();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public void stop() {
        drivebase.stop();
    }

    public double unnormalizeHeading(double angle) {
        if (previousPosition == null) {
            previousPosition = currentPosition;
        }

        if (previousPosition.getHeading(AngleUnit.DEGREES) > 90 &&
                previousPosition.getHeading(AngleUnit.DEGREES) < 181 &&
                currentPosition.getHeading(AngleUnit.DEGREES) < -90 && currentPosition.getHeading(AngleUnit.DEGREES) > -181) {
            // Flipped from 180 to -180
            headingWrapMultiplier += 1;
        } else if (previousPosition.getHeading(AngleUnit.DEGREES) < -90 &&
                previousPosition.getHeading(AngleUnit.DEGREES) > -181 &&
                currentPosition.getHeading(AngleUnit.DEGREES) > 90 && currentPosition.getHeading(AngleUnit.DEGREES) < 181) {
            // Flipped from -180 to 180
            headingWrapMultiplier -= 1;
        }

        previousPosition = currentPosition;

        return angle + (headingWrapMultiplier * 360);
    }

    public double[] headingCalculations(double currentHeading, double targetHeading) {
        double unnormalizedHeading = unnormalizeHeading(currentHeading);

        // Checks if headings are within bounds
//        if (unnormalizedHeading < -180 || unnormalizedHeading > 180 || targetHeading < -180 || targetHeading > 180) {
//            throw new IllegalArgumentException("Angles must be between -180 and 180");
//        }

        double distanceDirect = Math.abs(currentHeading - targetHeading);

        double distanceTo180FromCurrent = Math.min(Math.abs(unnormalizedHeading - 180), Math.abs(unnormalizedHeading + 180));
        double distanceTo180FromTarget = Math.min(Math.abs(targetHeading - 180), Math.abs(targetHeading + 180));
        double distanceCrossing180 = distanceTo180FromCurrent + distanceTo180FromTarget;

        if (distanceCrossing180 < distanceDirect) {
            // Crossing 180 degrees is faster
            // Trick the PID controller to go faster
            if (targetHeading > unnormalizedHeading) {
                targetHeading -= 360;
            } else {
                targetHeading += 360;
            }
        }

        return new double[]{unnormalizedHeading, targetHeading};
    }

    public Pose2D getPosition() {
        return currentPosition;
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
        currentPosition = pose;
    }

    public Command moveQuickly(double x, double y, double h) {
        return new QuickMoveTo(x, y, h);
    }

//    public Command turnQuickly(double heading) {
//        return new TurnMoveTo(heading);
//    }

    public class QuickMoveTo extends CommandBase {
        private final PIDFController quickStrafe;
        private final PIDFController quickForward;
        private final PIDFController quickTurn;

        // TODO: test quick move to with X, Y, AND HEADING PIDs in use, see result of that
        public QuickMoveTo(double x, double y, double h) {
            target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
            quickStrafe = new PIDController(STRAFE_PID_QUICK.p, STRAFE_PID_QUICK.i, STRAFE_PID_QUICK.d);
            quickForward = new PIDController(FORWARD_PID_QUICK.p, FORWARD_PID_QUICK.i, FORWARD_PID_QUICK.d);
            quickTurn = new PIDController(HEADING_PID_QUICK.p, HEADING_PID_QUICK.i, HEADING_PID_QUICK.d);
            quickStrafe.setTolerance(DISTANCE_TOLERANCE_LOW);
            quickForward.setTolerance(DISTANCE_TOLERANCE_LOW);
            quickTurn.setTolerance(ANGLE_TOLERANCE, ANGLE_VELOCITY_TOLERANCE);
            quickStrafe.setSetPoint(target.getX(DISTANCE_UNIT));
            quickForward.setSetPoint(target.getY(DISTANCE_UNIT));
            quickTurn.setSetPoint(target.getHeading(ANGLE_UNIT));
            addRequirements(Drive.this);
        }

        @Override
        public void initialize() {
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
        }

        @Override
        public void execute() {
            strafe = quickStrafe.calculate(currentPosition.getX(DISTANCE_UNIT), target.getX(DISTANCE_UNIT));
            forward = quickForward.calculate(currentPosition.getY(DISTANCE_UNIT), target.getY(DISTANCE_UNIT));
            turn = quickTurn.calculate(unnormalizeHeading(currentPosition.getHeading(ANGLE_UNIT)), target.getHeading(ANGLE_UNIT));

            // Custom Static Friction Calculations TODO: TUNE STATIC_F_SENSITIVE
            if (strafe > STATIC_F_SENSITIVE) ffStrafe = STRAFE_PID_QUICK.f;
            if (strafe < -STATIC_F_SENSITIVE) ffStrafe = -STRAFE_PID_QUICK.f;
            if (forward > STATIC_F_SENSITIVE) ffForward = FORWARD_PID_QUICK.f;
            if (forward < -STATIC_F_SENSITIVE) ffForward = -FORWARD_PID_QUICK.f;
            if (turn > STATIC_F_SENSITIVE) ffTurn = HEADING_PID_QUICK.f;
            if (turn < -STATIC_F_SENSITIVE) ffTurn = -HEADING_PID_QUICK.f;

            strafe += ffStrafe;
            forward += ffForward;
            turn += ffTurn;

            strafe = -strafe;
            turn = -turn;
        }

        @Override
        public boolean isFinished() {
            // Check if target is reached
            return quickStrafe.atSetPoint() && quickForward.atSetPoint() && quickTurn.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            strafe = 0;
            forward = 0;
            turn = 0;
            stop();
        }
    }

//    public class TurnMoveTo extends CommandBase {
//        private final PIDFController prevent_strafe;
//        private final PIDFController prevent_forward;
//        private final PIDFController quick_turn;
//
//        public TurnMoveTo(double h) {
//            target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, h);
//            prevent_strafe = new PIDController(0, 0, 0);
//            prevent_forward = new PIDController(0, 0, 0);
//            quick_turn = new PIDController(HEADING_PID_QUICK.p, HEADING_PID_QUICK.i, HEADING_PID_QUICK.d);
//            prevent_strafe.setTolerance(DISTANCE_TOLERANCE_LOW);
//            prevent_forward.setTolerance(DISTANCE_TOLERANCE_LOW);
//            quick_turn.setTolerance(ANGLE_TOLERANCE, ANGLE_VELOCITY_TOLERANCE);
//            prevent_strafe.setSetPoint(target.getX(DISTANCE_UNIT));
//            prevent_forward.setSetPoint(target.getY(DISTANCE_UNIT));
//            quick_turn.setSetPoint(target.getHeading(ANGLE_UNIT));
//            addRequirements(Drive.this);
//        }
//
//        @Override
//        public void initialize() {
//            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
//        }
//
//        @Override
//        public void execute() {
//            strafe = prevent_strafe.calculate(currentPosition.getX(DISTANCE_UNIT), target.getX(DISTANCE_UNIT));
//            forward = prevent_forward.calculate(currentPosition.getY(DISTANCE_UNIT), target.getY(DISTANCE_UNIT));
//            turn = quick_turn.calculate(unnormalizeHeading(currentPosition.getHeading(ANGLE_UNIT)), target.getHeading(ANGLE_UNIT));
//
//            // Custom Static Friction Calculations TODO: TUNE STATIC_F_SENSITIVE
//            if (strafe > STATIC_F_SENSITIVE) ffStrafe = STRAFE_PID_QUICK.f;
//            if (strafe < -STATIC_F_SENSITIVE) ffStrafe = -STRAFE_PID_QUICK.f;
//            if (forward > STATIC_F_SENSITIVE) ffForward = FORWARD_PID_QUICK.f;
//            if (forward < -STATIC_F_SENSITIVE) ffForward = -FORWARD_PID_QUICK.f;
//            // TODO: ADD STATIC FRICTION FOR HEADING
//            if (turn > STATIC_F_SENSITIVE) ffTurn = HEADING_PID_QUICK.f;
//            if (turn < -STATIC_F_SENSITIVE) ffTurn = -HEADING_PID_QUICK.f;
//
//            strafe += ffStrafe;
//            forward += ffForward;
//            turn += 0;
//        }
//
//        @Override
//        public boolean isFinished() {
//            // Check if target is reached
//            return prevent_strafe.atSetPoint() && prevent_forward.atSetPoint() && quick_turn.atSetPoint();
//        }
//
//        @Override
//        public void end(boolean interrupted) {
//            strafe = 0;
//            forward = 0;
//            turn = 0;
//            stop();
//        }
//    } fix later

    public class HumanInputs extends CommandBase {
        GamepadEx driver;
        private final PIDFController quickTurn;

        public HumanInputs(GamepadEx driver) {
            this.driver = driver;
            addRequirements(Drive.this);

            quickTurn = new PIDController(0.035, HEADING_PID_QUICK.i, HEADING_PID_QUICK.d);
            quickTurn.setTolerance(ANGLE_TOLERANCE, ANGLE_VELOCITY_TOLERANCE);
        }

        @Override
        public void execute() {
            // TODO: FIX HEADING LOCK AND PRECISE TURNING
            strafe = scaleInputs(driver.getRightX());
            forward = scaleInputs(-driver.getRightY());
//            strafe = -driver.getRightX();
//            forward = -driver.getRightY();
            double leftX = driver.getLeftX();
            if (Math.abs(leftX) > DEAD_ZONE)
                target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, target.getHeading(ANGLE_UNIT) - TURN_SPEED * leftX);
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, 0);
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, 180);
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, 90);
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, -90);

//            double[] headings = headingCalculations(currentPosition.getHeading(ANGLE_UNIT), target.getHeading(ANGLE_UNIT));
//            double currentHeading = headings[0];
//            double targetHeading = headings[1];
//            turn = quickTurn.calculate(currentHeading, targetHeading);
            turn = quickTurn.calculate(currentPosition.getHeading(ANGLE_UNIT), target.getHeading(ANGLE_UNIT));

            if (turn > STATIC_F_SENSITIVE) ffTurn = HEADING_PID_QUICK.f;
            if (turn < -STATIC_F_SENSITIVE) ffTurn = -HEADING_PID_QUICK.f;
            turn += ffTurn;
            turn = -turn;

            // Left trigger for turbo mode
            if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
                turbo(true);
            } else {
                turbo(false);
            }
        }

        public double scaleInputs(double input) {
            if (Math.abs(input) > DEAD_ZONE)
                return Math.pow(Math.abs(input), 2) * Math.signum(input);
            else
                return 0;
        }
    }

    public void turbo(boolean on) {
        if (on) {
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
        } else {
            drivebase.setMaxSpeed(TURBO_SLOW_SPEED);
        }
    }

    public void readSensors() {
        // Get the latest pose
//        currentPosition = pinpoint.getPosition();
        Pose2D pinpointPosition = pinpoint.getPosition();
        currentPosition = new Pose2D(DISTANCE_UNIT, pinpointPosition.getX(DISTANCE_UNIT), pinpointPosition.getY(DISTANCE_UNIT), ANGLE_UNIT, -pinpointPosition.getHeading(ANGLE_UNIT));
    }

    @Override
    public void periodic() {
        pinpoint.update();
        drivebase.driveFieldCentric(strafe, forward, turn, currentPosition.getHeading(ANGLE_UNIT));
    }

    public void addTelemetry(TelemetryPacket pack) {
        double circleRadius = 10;
        // Angle theta in radians (example angle, can be calculated dynamically)
        double theta = -currentPosition.getHeading(AngleUnit.RADIANS);

        // Length of the red line (in pixels)
        double lineLength = 10;

        // Calculate coordinates of the red line endpoints
        double x1 = currentPosition.getY(DISTANCE_UNIT) + (circleRadius - lineLength) * Math.cos(theta);
        double y1 = -currentPosition.getX(DISTANCE_UNIT) + (circleRadius - lineLength) * Math.sin(theta);

        double x2 = currentPosition.getY(DISTANCE_UNIT) + circleRadius * Math.cos(theta);
        double y2 = -currentPosition.getX(DISTANCE_UNIT) + circleRadius * Math.sin(theta);

        pack.put("Current X", currentPosition.getX(DISTANCE_UNIT));
        pack.put("Current Y", currentPosition.getY(DISTANCE_UNIT));
        pack.put("Target X", target.getX(DISTANCE_UNIT));
        pack.put("Target Y", target.getY(DISTANCE_UNIT));
        pack.put("Target Heading", target.getHeading(ANGLE_UNIT));
        pack.put("Current Heading", currentPosition.getHeading(ANGLE_UNIT));
        pack.put("Current Heading Unnormalized", unnormalizeHeading(currentPosition.getHeading(ANGLE_UNIT)));
        pack.put("Strafe Power", strafe);
        //pack.put("Forward Power", forward); FIXME
        pack.put("Strafe Friction", ffStrafe);
        pack.put("Forward Friction", ffForward);
        //pack.put("Turn Power", turn); FIXME
        pack.field().drawImage("", 0,0,144,144);
        pack.fieldOverlay().strokeCircle(currentPosition.getY(DISTANCE_UNIT), -currentPosition.getX(DISTANCE_UNIT), circleRadius).strokeLine(x1, y1, x2, y2);
    }
}

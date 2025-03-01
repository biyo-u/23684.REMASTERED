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
    public static PIDFCoefficients HEADING_PID_QUICK = new PIDFCoefficients(0.04, 0, 0, 0);
    public static PIDFCoefficients FORWARD_PID_QUICK = new PIDFCoefficients(0.04, 0, 0.005, 0.0045);
    public static PIDFCoefficients STRAFE_PID_QUICK = new PIDFCoefficients(0.055, 0, 0, 0);
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static double DISTANCE_TOLERANCE_LOW = 0.01;
    public static double ANGLE_TOLERANCE = 1;
    public static double ANGLE_VELOCITY_TOLERANCE = Double.POSITIVE_INFINITY;
    public static double DEAD_ZONE = 0.1;
    public static double TURN_SPEED = 3;
    public static double TURBO_FAST_SPEED = 1.0;
    public static double TURBO_SLOW_SPEED = 0.75;
    public static double STATIC_F_SENSITIVE = 0.005;
    MecanumDrive drivebase;
    GoBildaPinpointDriver pinpoint;
    double forward;
    double strafe;
    double turn;
    double ffForward;
    double ffStrafe;
    double ffTurn;
    public static Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
    Pose2D currentPosition;
    Pose2D previousPosition;
    double headingWrapMultiplier = 0;
    Pose2D target = new Pose2D(DISTANCE_UNIT,0,0, ANGLE_UNIT,0);

    // NOTE: we give "Drive" access to "Arm" here so that our Driver can initiate climbing
    //public Arm arm = null; TODO: CREATE ARM!!!!!!!!!!!!!

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
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
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
                currentPosition.getHeading(AngleUnit.DEGREES) < -90 && currentPosition.getHeading(AngleUnit.DEGREES) > -181){
            // Flipped from 180 to -180
            headingWrapMultiplier += 1;
        } else if (previousPosition.getHeading(AngleUnit.DEGREES) < -90 &&
                previousPosition.getHeading(AngleUnit.DEGREES) > -181 &&
                currentPosition.getHeading(AngleUnit.DEGREES) > 90 && currentPosition.getHeading(AngleUnit.DEGREES) < 181){
            // Flipped from -180 to 180
            headingWrapMultiplier -= 1;
        }

        previousPosition = currentPosition;

        return angle + (headingWrapMultiplier * 360);
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
        currentPosition = pose;
    }

    public Pose2D getPosition() {
        return currentPosition;
    }

    public Command moveQuickly(double x, double y, double heading) {
        return new QuickMoveTo(x, y, heading);
    }

    public Command turnQuickly(double x, double y, double heading) {
        return new TurnMoveTo(x, y, heading);
    }

    public class QuickMoveTo extends CommandBase {
        private final PIDFController quickStrafe;
        private final PIDFController quickForward;
        private final PIDFController quickTurn;

        public QuickMoveTo(double x, double y, double h) {
            target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
            quickStrafe = new PIDController(STRAFE_PID_QUICK.p, STRAFE_PID_QUICK.i, STRAFE_PID_QUICK.d);
            quickForward = new PIDController(FORWARD_PID_QUICK.p, FORWARD_PID_QUICK.i, FORWARD_PID_QUICK.d);
            quickTurn = new PIDController(0, 0, 0);
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
            // TODO: ADD STATIC FRICTION FOR HEADING
            if (turn > STATIC_F_SENSITIVE) ffTurn = HEADING_PID_QUICK.f;
            if (turn < -STATIC_F_SENSITIVE) ffTurn = -HEADING_PID_QUICK.f;

            strafe += ffStrafe;
            forward += ffForward;
            turn += 0;
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

    public class TurnMoveTo extends CommandBase {
        private final PIDFController prevent_strafe;
        private final PIDFController prevent_forward;
        private final PIDFController quick_turn;

        public TurnMoveTo(double x, double y, double h) {
            target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
            prevent_strafe = new PIDController(0, 0, 0);
            prevent_forward = new PIDController(0, 0, 0);
            quick_turn = new PIDController(HEADING_PID_QUICK.p, HEADING_PID_QUICK.i, HEADING_PID_QUICK.d);
            prevent_strafe.setTolerance(DISTANCE_TOLERANCE_LOW);
            prevent_forward.setTolerance(DISTANCE_TOLERANCE_LOW);
            quick_turn.setTolerance(ANGLE_TOLERANCE, ANGLE_VELOCITY_TOLERANCE);
            prevent_strafe.setSetPoint(target.getX(DISTANCE_UNIT));
            prevent_forward.setSetPoint(target.getY(DISTANCE_UNIT));
            quick_turn.setSetPoint(target.getHeading(ANGLE_UNIT));
            addRequirements(Drive.this);
        }

        @Override
        public void initialize() {
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
        }

        @Override
        public void execute() {
            strafe = prevent_strafe.calculate(currentPosition.getX(DISTANCE_UNIT), target.getX(DISTANCE_UNIT));
            forward = prevent_forward.calculate(currentPosition.getY(DISTANCE_UNIT), target.getY(DISTANCE_UNIT));
            turn = quick_turn.calculate(unnormalizeHeading(currentPosition.getHeading(ANGLE_UNIT)), target.getHeading(ANGLE_UNIT));

            // Custom Static Friction Calculations TODO: TUNE STATIC_F_SENSITIVE
            if (strafe > STATIC_F_SENSITIVE) ffStrafe = STRAFE_PID_QUICK.f;
            if (strafe < -STATIC_F_SENSITIVE) ffStrafe = -STRAFE_PID_QUICK.f;
            if (forward > STATIC_F_SENSITIVE) ffForward = FORWARD_PID_QUICK.f;
            if (forward < -STATIC_F_SENSITIVE) ffForward = -FORWARD_PID_QUICK.f;
            // TODO: ADD STATIC FRICTION FOR HEADING
            if (turn > STATIC_F_SENSITIVE) ffTurn = HEADING_PID_QUICK.f;
            if (turn < -STATIC_F_SENSITIVE) ffTurn = -HEADING_PID_QUICK.f;

            strafe += ffStrafe;
            forward += ffForward;
            turn += 0;
        }

        @Override
        public boolean isFinished() {
            // Check if target is reached
            return prevent_strafe.atSetPoint() && prevent_forward.atSetPoint() && quick_turn.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            strafe = 0;
            forward = 0;
            turn = 0;
            stop();
        }
    }

    public class HumanInputs extends CommandBase {
        GamepadEx driver;

        public HumanInputs(GamepadEx driver) {
            this.driver = driver;
            addRequirements(Drive.this);
        }

        @Override
        public void execute() {
//            strafe = scaleInputs(driver.getRightX());
//            forward = scaleInputs(-driver.getRightY());
            strafe = driver.getRightX();
            forward = -driver.getRightY();
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

            // Left trigger for turbo mode
//            if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
//                turbo(true);
//            } else {
//                turbo(false);
//            }
        }

        // TODO: Re-add this to HumanInputs
//        public double scaleInputs(double input) {
//            if (Math.abs(input) > DEAD_ZONE)
//                return Math.pow(Math.abs(input), 2) * Math.signum(input);
//            else
//                return 0;
//        }
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
        currentPosition = pinpoint.getPosition();
    }

    @Override
    public void periodic() {
        pinpoint.update();
        drivebase.driveFieldCentric(strafe, forward, turn, currentPosition.getHeading(ANGLE_UNIT));
    }

    public void addTelemetry(TelemetryPacket pack) {
        pack.put("Current X", currentPosition.getX(DISTANCE_UNIT));
        pack.put("Current Y", currentPosition.getY(DISTANCE_UNIT));
        pack.put("Target X", target.getX(DISTANCE_UNIT));
        pack.put("Target Y", target.getY(DISTANCE_UNIT));
        pack.put("Target Heading", target.getHeading(ANGLE_UNIT));
        pack.put("Current Heading", currentPosition.getHeading(ANGLE_UNIT));
        pack.put("Current Heading Unnormalized", unnormalizeHeading(currentPosition.getHeading(ANGLE_UNIT)));
        pack.put("Strafe Power", strafe);
        pack.put("Forward Power", forward);
        pack.put("Strafe Friction", ffStrafe);
        pack.put("Forward Friction", ffForward);
        pack.put("Turn Power", turn);
    }
}

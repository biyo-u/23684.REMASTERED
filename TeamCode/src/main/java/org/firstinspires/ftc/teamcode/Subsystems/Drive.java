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

@Config
public class Drive extends SubsystemBase {
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static double DISTANCE_TOLERANCE_LOW = 0.01;
    public static double ANGLE_TOLERANCE = 1;
    public static double ANGLE_VELOCITY_TOLERANCE = Double.POSITIVE_INFINITY;
    public static double POWER_INPUT = 2;
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
    double ff_forward;
    double ff_strafe;
    double ff_turn;
    public static PIDFCoefficients heading_pid_quick = new PIDFCoefficients(0.04, 0, 0, 0);
    public static PIDFCoefficients forward_pid_quick = new PIDFCoefficients(0.04, 0, 0.005, 0.0045);
    public static PIDFCoefficients strafe_pid_quick = new PIDFCoefficients(0.055, 0, 0, 0);
    public static Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
    Pose2D current_position;
    Pose2D previous_position;
    double headingWrapMultiplier = 0;
    Pose2D target = new Pose2D(DISTANCE_UNIT,0,0, ANGLE_UNIT,0);

    // NOTE: we give "Drive" access to "Arm" here so that our Driver can initiate climbing
    //public Arm arm = null; TODO: CREATE ARM!!!!!!!!!!!!!

    public Drive(HardwareMap hardwareMap) {
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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        this.pinpoint.setOffsets(-173.0, 156);
        this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.pinpoint.resetPosAndIMU();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public void stop() {
        drivebase.stop();
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
        current_position = pose;
    }

    public Pose2D getPosition() {
        return current_position;
    }

    public Command moveQuickly(double x, double y, double heading) {
        return new QuickMoveTo(x, y, heading);
    }

    public class QuickMoveTo extends CommandBase {
        private final PIDFController quick_strafe;
        private final PIDFController quick_forward;
        private final PIDFController quick_turn;

        public QuickMoveTo(double x, double y, double h) {
            target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
            quick_strafe = new PIDController(strafe_pid_quick.p, strafe_pid_quick.i, strafe_pid_quick.d);
            quick_forward = new PIDController(forward_pid_quick.p, forward_pid_quick.i, forward_pid_quick.d);
            quick_turn = new PIDController(heading_pid_quick.p, heading_pid_quick.i, heading_pid_quick.d);
            quick_strafe.setTolerance(DISTANCE_TOLERANCE_LOW);
            quick_forward.setTolerance(DISTANCE_TOLERANCE_LOW);
            quick_turn.setTolerance(ANGLE_TOLERANCE, ANGLE_VELOCITY_TOLERANCE);
            addRequirements(Drive.this);
        }

        @Override
        public void initialize() {
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
        }

        @Override
        public void execute() {
            // compute the direction vector relatively to the robot coordinates
            strafe = quick_strafe.calculate(current_position.getX(DISTANCE_UNIT), target.getX(DISTANCE_UNIT));
            forward = quick_forward.calculate(current_position.getY(DISTANCE_UNIT), target.getY(DISTANCE_UNIT));
            turn = quick_turn.calculate(unnormalizeHeading(current_position.getHeading(ANGLE_UNIT)), target.getHeading(ANGLE_UNIT));

            // Custom Static Friction Calculations TODO: TUNE STATIC_F_SENSITIVE
            if (strafe > STATIC_F_SENSITIVE) ff_strafe = strafe_pid_quick.f;
            if (strafe < -STATIC_F_SENSITIVE) ff_strafe = -strafe_pid_quick.f;
            if (forward > STATIC_F_SENSITIVE) ff_forward = forward_pid_quick.f;
            if (forward < -STATIC_F_SENSITIVE) ff_forward = -forward_pid_quick.f;
            // TODO: ADD STATIC FRICTION FOR HEADING
            if (turn > STATIC_F_SENSITIVE) ff_turn = heading_pid_quick.f;
            if (turn < -STATIC_F_SENSITIVE) ff_turn = -heading_pid_quick.f;

            strafe += ff_strafe;
            forward += ff_forward;
            turn += 0;
        }

        @Override
        public boolean isFinished() {
            // Check if target is reached
            return quick_strafe.atSetPoint() && quick_forward.atSetPoint() && quick_turn.atSetPoint();
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
            strafe = scaleInputs(driver.getRightX());
            forward = scaleInputs(-driver.getRightY());
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
        } else {
            drivebase.setMaxSpeed(TURBO_SLOW_SPEED);
        }
    }

    public void read_sensors(double time) {
        // Get the latest pose
        current_position = pinpoint.getPosition();
    }

    @Override
    public void periodic() {
        pinpoint.update();
        drivebase.driveFieldCentric(strafe, forward, turn, current_position.getHeading(ANGLE_UNIT));
    }

    public void add_telemetry(TelemetryPacket pack) {
        pack.put("Current X", current_position.getX(DISTANCE_UNIT));
        pack.put("Current Y", current_position.getY(DISTANCE_UNIT));
        pack.put("Target X", target.getX(DISTANCE_UNIT));
        pack.put("Target Y", target.getY(DISTANCE_UNIT));
        pack.put("Target Heading", target.getHeading(ANGLE_UNIT));
        pack.put("Current Heading",current_position.getHeading(ANGLE_UNIT));
        pack.put("Current Heading Unnormalized", unnormalizeHeading(current_position.getHeading(ANGLE_UNIT)));
        pack.put("Strafe Power", strafe);
        pack.put("Forward Power", forward);
        pack.put("Strafe Friction", ff_strafe);
        pack.put("Forward Friction", ff_forward);
        pack.put("Turn Power", turn);
    }
}

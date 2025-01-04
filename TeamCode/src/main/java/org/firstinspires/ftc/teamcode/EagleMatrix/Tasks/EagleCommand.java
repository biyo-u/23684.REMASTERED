package org.firstinspires.ftc.teamcode.EagleMatrix.Tasks;

import com.acmerobotics.dashboard.config.Config;
// FTCLIB Command Imports
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
// FTCLIB Drive Imports
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
// FTCLIB Gamepad Imports
// Hardware Imports
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
// Navigation Imports
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// Utilities Imports
import org.firstinspires.ftc.teamcode.Utilities.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

// so basically im just copying the hyperdroid code line-by-line and adapting it to our set up.
@Config
public class EagleCommand extends SubsystemBase {
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES; // Sets unit angles are read in globally
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH; // Sets unit distances are read in globally
    public static double DISTANCE_ERROR_TOLERANCE = 0.005; // how much error is tolerated between robot location and target location, in DISTANCE_UNITs to target,
    public static double ANGLE_ERROR_TOLERANCE = 5; // how much error is tolerated between robot heading and target heading, in ANGLE_UNITs to target
    public static double DEAD_ZONE = 0.1; //might remove
    public static double SPEED_MODIFIER = 0.75;
    public static double DRIVE_SPEED = 1.0;

    Robot robot; //calls in Robot.java
    MecanumDrive ZETA_PRIME;

    // inputs into the drivebase, from human or auto
    double Y_DRIVE; // +Fwd/-Rev (moving forward or backward)
    double X_DRIVE; // +Right/-Left (strafing left or right)
    double H_DRIVE; // +CW/-CCW (turning clockwise or counterclockwise)
    double GOAL_HEADING;

    private final PIDFController HEADING;
    private final PIDFController STRAFING;
    private final PIDFController FORWARDING;

    public static PIDFCoefficients PID = new PIDFCoefficients(Constants.PIDValues.KP, Constants.PIDValues.KI, Constants.PIDValues.KD, Constants.PIDValues.KF); //adjusted November 1
    public static PIDFCoefficients anglePID = new PIDFCoefficients(Constants.PIDValues.HEADING_P, Constants.PIDValues.HEADING_I, Constants.PIDValues.HEADING_D, Constants.PIDValues.HEADING_F);
    public static Motor.ZeroPowerBehavior ZPB = Motor.ZeroPowerBehavior.BRAKE; // sets the zero power behavior of the motors
    Pose2D current_position;

    public EagleCommand(Robot robot, HardwareMap hardwareMap) {
        this.robot = robot;

        Motor FL = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        FL.setInverted(true);
        FL.setZeroPowerBehavior(ZPB);

        Motor FR = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        FR.setZeroPowerBehavior(ZPB);

        Motor RL = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
        RL.setInverted(true);
        RL.setZeroPowerBehavior(ZPB);

        Motor RR = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        RR.setZeroPowerBehavior(ZPB);

        ZETA_PRIME = new MecanumDrive(false, FL, FR, RL, RR);
        ZETA_PRIME.setMaxSpeed(1);

        FORWARDING = new PIDFController(PID.p, PID.i, PID.d, PID.f);
        FORWARDING.setTolerance(DISTANCE_ERROR_TOLERANCE, Double.POSITIVE_INFINITY);

        STRAFING = new PIDFController(PID.p, PID.i, PID.d, PID.f);
        STRAFING.setTolerance(DISTANCE_ERROR_TOLERANCE, Double.POSITIVE_INFINITY);

        HEADING = new PIDFController(anglePID.p, anglePID.i, anglePID.d, anglePID.f);
        HEADING.setTolerance(ANGLE_ERROR_TOLERANCE, Double.POSITIVE_INFINITY);
    }

    public void stop() {
        ZETA_PRIME.stop();
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
        GOAL_HEADING = pose.getHeading(AngleUnit.DEGREES);
    }

    public Pose2D getPosition() {
        return current_position;
    }

    public Pose2D getTarget() {
        Pose2D findTarget = new Pose2D(DISTANCE_UNIT, STRAFING.getSetPoint(), FORWARDING.getSetPoint(), ANGLE_UNIT, GOAL_HEADING);
        return findTarget;
    }

    public Command moveTo(double x, double y, double h, double speed, double error) {
        return moveTo(x, y, h, (DRIVE_SPEED * 0.5), DISTANCE_ERROR_TOLERANCE);
    }

    public class MoveTo extends CommandBase {
        Pose2D target;
        double speed;
        double error;

        public MoveTo(double x, double y, double h, double speed, double error) {
            target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
            this.speed = speed;
            this.error = error;
            addRequirements(EagleCommand.this);
        }

        public void setTarget(double x_goal, double y_goal, double h_goal) {
            STRAFING.setSetPoint(x_goal);
            FORWARDING.setSetPoint(y_goal);
            HEADING.setSetPoint(wrapAngle(h_goal));
        }

        @Override
        public void initialize() {
            setTarget(target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), target.getHeading(ANGLE_UNIT));
            FORWARDING.setTolerance(DISTANCE_ERROR_TOLERANCE, Double.POSITIVE_INFINITY);
            STRAFING.setTolerance(DISTANCE_ERROR_TOLERANCE, Double.POSITIVE_INFINITY);
            ZETA_PRIME.setMaxSpeed(speed);
        }

        @Override
        public void execute() {
            Y_DRIVE = FORWARDING.calculate(current_position.getY(DISTANCE_UNIT));
            X_DRIVE = STRAFING.calculate(current_position.getX(DISTANCE_UNIT));
        }

        @Override
        public boolean isFinished() {
            return FORWARDING.atSetPoint() && STRAFING.atSetPoint() && HEADING.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            Y_DRIVE = 0;
            X_DRIVE = 0;
            stop();
        }
    }

    public void turbo(boolean activated) {
        if (activated) {
            ZETA_PRIME.setMaxSpeed(1.0);
        } else {
            ZETA_PRIME.setMaxSpeed(0.75);
        }
    }

    public void read_sensors() {
        // Get the latest pose, which includes the x and y coordinates, plus the heading angle
        current_position = robot.odometry.getPosition();
    }


}

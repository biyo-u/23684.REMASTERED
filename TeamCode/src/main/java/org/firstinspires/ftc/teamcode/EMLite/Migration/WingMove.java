package org.firstinspires.ftc.teamcode.EMLite.Migration;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

public class WingMove {
    Robot robot;
    double modifier = 0.6;
    double rest = 0.0;

    public enum MotorDirection {
        FORWARD(1, 1, 1, 1),
        BACKWARD(-1, -1, -1, -1),
        STRAFE_LEFT(-1, 1, 1, -1),
        STRAFE_RIGHT(1, -1, -1, 1),
        ANGLE_STRAFE_FORWARD_RIGHT(1, 0, 0, 1),
        ANGLE_STRAFE_FORWARD_LEFT(0, 1, 1, 0),
        ANGLE_STRAFE_BACKWARD_RIGHT(-1, 0, 0, -1),
        ANGLE_STRAFE_BACKWARD_LEFT(0, -1, -1, 0),
        ROTATE_CLOCKWISE(1, -1, 1, -1),
        ROTATE_COUNTERCLOCKWISE(-1, 1, -1, 1),
        STOP(0, 0, 0, 0);

        private final double FL;
        private final double FR;
        private final double RL;
        private final double RR;

        MotorDirection(double FL, double FR, double RL, double RR) {
            this.FL = FL;
            this.FR = FR;
            this.RL = RL;
            this.RR = RR;
        }
    }

    public WingMove(Robot robot) {
        this.robot = robot;
    }

    public void move(MotorDirection direction){
        robot.drive.getFrontLeft().setPower(direction.FL * modifier);
        robot.drive.getFrontRight().setPower(direction.FR * modifier);
        robot.drive.getRearLeft().setPower(direction.RL * modifier);
        robot.drive.getRearRight().setPower(direction.RR * modifier);
    }
}

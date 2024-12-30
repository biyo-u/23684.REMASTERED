package org.firstinspires.ftc.teamcode.EagleMatrix.Tasks;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.EagleMatrix.PIDDriver;
import org.firstinspires.ftc.teamcode.Utilities.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Task;

public class TurnPIDTask extends Task {

    PIDDriver PIDDriverYaw;

    public TurnPIDTask (double target, Robot robot) {
        PIDDriverYaw = new PIDDriver(robot.odometry.getHeading());
    }
    @Override
    public boolean run() {
        return false;
    }
}

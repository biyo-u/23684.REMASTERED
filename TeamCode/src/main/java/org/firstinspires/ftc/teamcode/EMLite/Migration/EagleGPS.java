package org.firstinspires.ftc.teamcode.EMLite.Migration;

import org.firstinspires.ftc.teamcode.Utilities.Robot;
import org.firstinspires.ftc.teamcode.EMLite.Migration.WingMove.MotorDirection;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.EMLite.Air_Traffic_Control;

public class EagleGPS {
    private final Robot robot; // imports robot hardwareMap class
    private final GoBildaPinpointDriver odometry; // imports robot odometry class
    private final WingMove wingmove; // imports EagleMatrix movement class for drivetrain
    public Air_Traffic_Control coordinates;
    public boolean GoalMet = false; // checks to see if goal (zetaTranslation) has been reached

    // lines 31 to 36 create all the numerical variables that will be used to compare due to the Double.compare function not working with the direct doubles
    double zetaY = 0; // robot's current X position cast to a double
    double zetaX = 0; // robot's current Y position cast to a double
    double zetaHeading = 0; // robot's current heading cast to a double

    // lines 45-47 create all the string variables that will be used to a) create the different values for each switch line of code, and keep track of AUTO as it runs
    String actionCounter = "PREP TO SCORE"; // robot's current action, String (text) value for switch block
    String autoStage = null; // robot's current action, String (text) value for telemetry
    String autoStatus = null; // robot's current action status, String (text) value for telemetry
    double correctionValue = 5.0;
    boolean tasksRun = false;

    public EagleGPS(Robot robot, WingMove wingmove, GoBildaPinpointDriver odometry, Air_Traffic_Control coordinates) {
        this.wingmove = wingmove;
        this.robot = robot;
        this.odometry = odometry;
        this.coordinates = coordinates;
    }

    public void prep_to_score() {
        autoStage = "STAGE: PREP TO SCORE";
        wingmove.clawclose();
        tasksRun = true;

        if (tasksRun == true) {
            actionCounter = "MOVE TO NET ZONE";
        }
    }

    public void move_to_net_zone() {
        autoStage = "STAGE: MOVE TO NET ZONE";

        if (zetaY - Air_Traffic_Control.driveToNetZone.y > correctionValue) {
            // if compare returns a negative value, value1 > value2
            autoStatus = "NEEDS TO MOVE BACK";
            wingmove.move(MotorDirection.BACKWARD);
        } else if (zetaY - Air_Traffic_Control.driveToNetZone.y < -correctionValue && !GoalMet) {
            // if compare returns a positive value, value1 < value2
            autoStatus = "NEEDS TO MOVE FORWARD";
            wingmove.move(MotorDirection.FORWARD);
            GoalMet = false;

        } else {
            // if compare returns a zero value, value1 == value2
            autoStatus = "COMPLETE";
            wingmove.move(MotorDirection.STOP);
            GoalMet = true;
        }

        if (GoalMet == true) {
            actionCounter = "PLACE PRELOADED SPECIMEN";
        }
    }

    public void place_preloaded_sample() {
        autoStage = "STAGE: PLACE PRELOADED SAMPLE";
        boolean taskNotDone = true;

        if (taskNotDone) {
            wingmove.clawopen();
            taskNotDone = false;
        } else if (taskNotDone == false) {
            actionCounter = "MOVE TO OBSERVATION ZONE";
        }
    }

    public void move_to_observation_zone() {
        autoStage = "STAGE: MOVE TO OBSERVATION ZONE";

        boolean FinalGoalMet = false;

        if (zetaY - Air_Traffic_Control.driveToObservationZone.y > correctionValue) {
            // if compare returns a negative value, value1 > value2
            autoStatus = "OVERSHOT, BUT IT'S ALRIGHT";
            wingmove.move(MotorDirection.STOP);
            FinalGoalMet = true;
        } else if (zetaY - Air_Traffic_Control.driveToObservationZone.y < -correctionValue && FinalGoalMet == false) {
            // if compare returns a positive value, value1 < value2
            autoStatus = "INCOMPLETE";
            wingmove.move(MotorDirection.BACKWARD);
        } else {
            autoStatus = "COMPLETE";
            wingmove.move(MotorDirection.STOP);
            FinalGoalMet = true;
        }

        if (FinalGoalMet == true) {
            wingmove.clawclose();
            actionCounter = "EXEUNT";
        }
    }

    public void exeunt() {
        autoStage = "STAGE: EXEUNT";
        autoStatus = "AUTONOMOUS PROCESS COMPLETE, PLEASE PREPARE FOR TELEOP.";
    }

    public void error() {
        autoStage = "ERROR";
    }

    public void last_resort() {
        autoStage = "STAGE: MOVE TO OBSERVATION ZONE";

        boolean GOGOGO = false;
        double moveanddontstop = -40;

        if (zetaY - moveanddontstop > correctionValue) {
            // if compare returns a negative value, value1 > value2
            autoStatus = "OVERSHOT, BUT IT'S ALRIGHT";
            wingmove.move(MotorDirection.STOP);
            GOGOGO = true;
        } else if (zetaY - moveanddontstop < -correctionValue && GOGOGO == false) {
            // if compare returns a positive value, value1 < value2
            autoStatus = "INCOMPLETE";
            wingmove.move(MotorDirection.BACKWARD);
        } else {
            autoStatus = "COMPLETE";
            wingmove.move(MotorDirection.STOP);
            GOGOGO = true;
        }

        if (GOGOGO == true) {
            actionCounter = "EXEUNT";
        }
    }
}


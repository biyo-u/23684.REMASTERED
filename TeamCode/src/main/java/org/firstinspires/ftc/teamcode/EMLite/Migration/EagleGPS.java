package org.firstinspires.ftc.teamcode.EMLite.Migration;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilities.Robot;
import org.firstinspires.ftc.teamcode.EMLite.Migration.WingMove.MotorDirection;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Old.Position;
import org.firstinspires.ftc.teamcode.Utilities.Old.Rotation;
import org.firstinspires.ftc.teamcode.Utilities.Old.Distance;

import java.util.Timer;
import java.util.TimerTask;

public class EagleGPS {
    private Robot robot; // imports robot hardwareMap class
    private GoBildaPinpointDriver odometry; // imports robot odometry class
    private WingMove wingmove; // imports EagleMatrix movement class for drivetrain
    public Position migration = new Position(new Distance(0, DistanceUnit.INCH), new Distance(20, DistanceUnit.INCH), new Rotation(180, AngleUnit.DEGREES)); // target position
    int counter; // counter to ensure AUTO program is active and running loops
    public boolean GoalMet = false; // checks to see if goal (zetaTranslation) has been reached

    // lines 31 to 36 create all the numerical variables that will be used to compare due to the Double.compare function not working with the direct doubles
    double zetaY = 0; // robot's current X position cast to a double
    double zetaY2 = 0; // robot's target X position cast to a double
    double zetaX = 0; // robot's current Y position cast to a double
    double zetaX2 = 0; // robot's target Y position cast to a double
    double zetaHeading = 0; // robot's current heading cast to a double
    double zetaHeading2 = 0; // robot's target heading cast to a double

    // lines 45-47 create all the string variables that will be used to a) create the different values for each switch line of code, and keep track of AUTO as it runs
    String actionCounter = "PREP TO SCORE"; // robot's current action, String (text) value for switch block
    String autoStage = null; // robot's current action, String (text) value for telemetry
    String autoStatus = null; // robot's current action status, String (text) value for telemetry

    double correctionValue = 5.0;

    boolean tasksRun = false;

    public void prep_to_score() {
        autoStage = "STAGE: PREP TO SCORE";

        Timer timertwo = new Timer("Timer");

        TimerTask preptwo = new TimerTask() {
            @Override
            public void run() {
                tasksRun = false;
                actionCounter = "MOVE TO SUBMERSIBLE";
            }
        };

        TimerTask prepone = new TimerTask() {
            @Override
            public void run() {
                timertwo.schedule(preptwo, 3000);
            }
        };

        if (!tasksRun) {
            timertwo.schedule(prepone,1000);
            tasksRun = true;
        }
    }

    public void move_to_net_zone() {
        autoStage = "STAGE: MOVE TO NET ZONE";

        if (zetaY - migration.getY() > correctionValue) {
            // if compare returns a negative value, value1 > value2
            autoStatus = "OVERSHOT";
            wingmove.move(MotorDirection.BACKWARD);

        } else if (zetaY - migration.getY() < -correctionValue && !GoalMet) {
            // if compare returns a positive value, value1 < value2
            autoStatus = "INCOMPLETE";
            wingmove.move(MotorDirection.FORWARD);

        } else {
            // if compare returns a zero value, value1 == value2
            autoStatus = "COMPLETE";
            wingmove.move(MotorDirection.STOP);
            GoalMet = true;
        }
        actionCounter = "PLACE PRELOADED SPECIMEN";
    }

    public void place_preloaded_sample() {
        autoStage = "STAGE: PLACE PRELOADED SAMPLE";

        Timer timerone = new Timer("Timer");

        TimerTask waittwo = new TimerTask() {
            @Override
            public void run() {
            }
        };

        TimerTask waitone = new TimerTask() {
            @Override
            public void run() {
                timerone.schedule(waittwo, 2000);
            }
        };

        if (!tasksRun) {
            timerone.schedule(waitone,3000);
            tasksRun = true;
        }

        actionCounter = "MOVE TO OBSERVATION ZONE";
    }

    public void move_to_observation_zone() {
        autoStage = "STAGE: MOVE TO OBSERVATION ZONE";

        boolean FinalGoalMet = false;
        Position observationZone = new Position(new Distance(30, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

        double zetaX2_3 = observationZone.getX();
        zetaX2 = observationZone.getX();

        if (zetaX - observationZone.getX() > correctionValue) {
            // if compare returns a negative value, value1 > value2
            autoStatus = "OVERSHOT, BUT IT'S ALRIGHT";
            wingmove.move(MotorDirection.STOP);
            FinalGoalMet = true;
        } else if (zetaX - observationZone.getX() < -correctionValue && FinalGoalMet == false) {
            // if compare returns a positive value, value1 < value2
            autoStatus = "INCOMPLETE";
            wingmove.move(MotorDirection.STRAFE_RIGHT);
        } else {
            autoStatus = "COMPLETE";
            wingmove.move(MotorDirection.STOP);
            FinalGoalMet = true;
        }

        if (FinalGoalMet == true) {
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

    public void looping() {

        // WHAT WE NEED: 1 - move forward, 2 - place sample in net zone, 3 - move to observation zone, 4 - end

        switch (actionCounter) {

            case "PREP TO SCORE": // PREPARES ROBOT LIFT AND INTAKE TO SCORING POSITION

                break;

            case "MOVE TO NET ZONE": // ROBOT DRIVES TO SUBMERSIBLE POSITION

                break;

            case "PLACE PRELOADED SAMPLE": // ROBOT PLACES PRELOADED SAMPLE ON IN NET ZONE

                break;

            case "MOVE TO OBSERVATION ZONE": // MOVES ROBOT TO OBSERVATION ZONE FROM SUBMERSIBLE POSITION

                break;

            case "EXEUNT": // ENDS THE SWITCH STATEMENT ONCE ALL CASES ARE COMPLETED

                break;

            default: // IF actionCounter IS NOT DEFINED OR DEFINED TO A UNDEFINED STATE, PRINT ERROR
                autoStage = "ERROR";
        }

        counter++; // update counter for ever loop
        odometry.update(); // updates odometry every loop
    }
}


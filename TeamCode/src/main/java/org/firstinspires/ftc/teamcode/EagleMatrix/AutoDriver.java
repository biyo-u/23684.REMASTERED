package org.firstinspires.ftc.teamcode.EagleMatrix;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Old.Distance;
import org.firstinspires.ftc.teamcode.Utilities.Old.Position;
import org.firstinspires.ftc.teamcode.Utilities.Old.Rotation;
import org.firstinspires.ftc.teamcode.EMLite.Migration.WingMove;
public class AutoDriver {
    Robot robot;
    GoBildaPinpointDriver odometry;
    WingMove driveMovements;
    boolean initialized = false;
    Position startPosition;
    Position originalMovementDirections;
    Telemetry telemetry;

    public AutoDriver(Robot robot, GoBildaPinpointDriver odometry) {
        this.robot = robot;
        this.odometry = odometry;
        this.driveMovements = new WingMove(robot);
        this.telemetry = new Telemetry() {
            @Override
            public Item addData(String caption, String format, Object... args) {
                return null;
            }

            @Override
            public Item addData(String caption, Object value) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, Func<T> valueProducer) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, String format, Func<T> valueProducer) {
                return null;
            }

            @Override
            public boolean removeItem(Item item) {
                return false;
            }

            @Override
            public void clear() {

            }

            @Override
            public void clearAll() {

            }

            @Override
            public Object addAction(Runnable action) {
                return null;
            }

            @Override
            public boolean removeAction(Object token) {
                return false;
            }

            @Override
            public void speak(String text) {

            }

            @Override
            public void speak(String text, String languageCode, String countryCode) {

            }

            @Override
            public boolean update() {
                return false;
            }

            @Override
            public Line addLine() {
                return null;
            }

            @Override
            public Line addLine(String lineCaption) {
                return null;
            }

            @Override
            public boolean removeLine(Line line) {
                return false;
            }

            @Override
            public boolean isAutoClear() {
                return false;
            }

            @Override
            public void setAutoClear(boolean autoClear) {

            }

            @Override
            public int getMsTransmissionInterval() {
                return 0;
            }

            @Override
            public void setMsTransmissionInterval(int msTransmissionInterval) {

            }

            @Override
            public String getItemSeparator() {
                return "";
            }

            @Override
            public void setItemSeparator(String itemSeparator) {

            }

            @Override
            public String getCaptionValueSeparator() {
                return "";
            }

            @Override
            public void setCaptionValueSeparator(String captionValueSeparator) {

            }

            @Override
            public void setDisplayFormat(DisplayFormat displayFormat) {

            }

            @Override
            public Log log() {
                return null;
            }
        };
    }

    public boolean moveTo(Position targetposition, Distance distanceSensitivity, Rotation headingSensitivity) {
        odometry.update();
        Pose2D currentPositionPose = odometry.getPosition();
        Position currentPosition = new Position(new Distance(currentPositionPose.getX(DistanceUnit.INCH), DistanceUnit.INCH), new Distance(currentPositionPose.getY(DistanceUnit.INCH), DistanceUnit.INCH), new Rotation(currentPositionPose.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES));
        originalMovementDirections = new Position(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

        if (!initialized) {
            telemetry.addData("Initialized", "true");
            startPosition = currentPosition;
            if (startPosition.getX() == targetposition.getX()){
                originalMovementDirections.setX(new Distance(0, DistanceUnit.INCH));
            } else if (startPosition.getX() > targetposition.getX()){
                originalMovementDirections.setX(new Distance(-1, DistanceUnit.INCH));
            } else if (startPosition.getX() < targetposition.getX()){
                originalMovementDirections.setX(new Distance(1, DistanceUnit.INCH));
            }

            if (startPosition.getY() == targetposition.getY()){
                originalMovementDirections.setY(new Distance(0, DistanceUnit.INCH));
            } else if (startPosition.getY() > targetposition.getY()){
                originalMovementDirections.setY(new Distance(-1, DistanceUnit.INCH));
            } else if (startPosition.getY() < targetposition.getY()){
                originalMovementDirections.setY(new Distance(1, DistanceUnit.INCH));
            }

            if (startPosition.getHeading() == targetposition.getHeading()){
                originalMovementDirections.setHeading(0);
            } else if (startPosition.getY() > targetposition.getY()){
                originalMovementDirections.setHeading(-1);
            } else if (startPosition.getY() < targetposition.getY()){
                originalMovementDirections.setHeading(1);
            }
        }

        double x = 0;
        double y = 0;
        double heading = 0;

        double yDifferenceFromTarget = Math.abs(currentPosition.getY()) - Math.abs(targetposition.getVector2D().getY());
        if (yDifferenceFromTarget > -distanceSensitivity.getDistance() && yDifferenceFromTarget < distanceSensitivity.getDistance()){
            // No movement needed on y axis
            y = 0;
        } else if (targetposition.getY() > currentPosition.getY()){
            // Move forward
            y = 1;
        } else if (targetposition.getY() < currentPosition.getY()){
            // Move backward
            y = -1;
        }

        double xDifferenceFromTarget = Math.abs(currentPosition.getX()) - Math.abs(targetposition.getVector2D().getX());
        if (xDifferenceFromTarget > -distanceSensitivity.getDistance() && xDifferenceFromTarget < distanceSensitivity.getDistance()){
            // No movement on the x axis needed
            x = 0;
        } else if (targetposition.getX() > currentPosition.getX()){
            // Move right
            x = 1;
        } else if (targetposition.getX() < currentPosition.getX()){
            // Move left
            x = -1;
        }

        double headingDifferenceFromTarget = Math.abs(currentPosition.getHeading()) - Math.abs(targetposition.getHeading());
        if (headingDifferenceFromTarget > -headingSensitivity.getAngle() && headingDifferenceFromTarget < headingSensitivity.getAngle()){
            // No turning needed
            heading = 0;
        } else if (targetposition.getHeading() > currentPosition.getHeading()){
            // Turn more
            heading = 1;
        } else if (targetposition.getHeading() < currentPosition.getHeading()){
            // Turn less
            heading = -1;
        }

		/* Speed Formulas (in LaTeX):
		Normal: y=\frac{\left(\ \frac{1}{\sqrt{2\pi}}e^{-\frac{1}{2}\left(\frac{x-0.5}{0.3}\right)^{2}}\right)}{1}
		Overshot: y=-\frac{\left(\ \frac{1}{\sqrt{2\pi}}e^{-\frac{1}{2}\left(\frac{x-1.5}{0.3}\right)^{2}}\right)}{1}
		 */

        double xProgress = (targetposition.getX() - startPosition.getX())/startPosition.getX();
        double xSpeed = 0;
        if (xProgress < 1){
            xSpeed = ((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((xProgress - 0.5) / 0.2), 2.0)))) * 1);
        } else if (xProgress == 1) {
            xSpeed = 0;
        } else {
            xSpeed = -((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((xProgress - 1.5) / 0.2), 2.0)))) * 1);
        }

        double yProgress = (targetposition.getY() - startPosition.getY())/startPosition.getY();
        double ySpeed = 0;
        if (yProgress < 1){
            ySpeed = ((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((yProgress - 0.5) / 0.2), 2.0)))) * 1);
        } else if (yProgress == 1) {
            ySpeed = 0;
        } else {
            ySpeed = -((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((yProgress - 1.5) / 0.2), 2.0)))) * 1);
        }

        double headingProgress = (targetposition.getHeading() - startPosition.getHeading())/startPosition.getHeading();
        double headingSpeed = 0;
        if (headingProgress < 0.95){
            // TODO: URGENT!!!!! REPLACE HEADING MATH WITH U.M.M EQUATION
            headingSpeed = ((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((headingProgress - 0.5) / 0.2), 2.0)))) * 1);
        } else if (headingProgress > 0.95 && headingProgress < 1.05) {
            headingSpeed = 0;
        } else {
            // TODO: URGENT!!!!! REPLACE HEADING MATH WITH U.M.M EQUATION
            headingSpeed = -((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((headingProgress - 1.5) / 0.2), 2.0)))) * 1);
        }

        double xPower = x * (xSpeed + 0.3);
        double yPower = y * (ySpeed + 0.3);
        double headingPower = heading * (headingSpeed + 0.3);

        double currentHeading = currentPosition.getHeading();

        double rotX = xPower * Math.cos(-currentHeading) - yPower * Math.sin(-currentHeading);
        double rotY = xPower * Math.sin(-currentHeading) + yPower * Math.cos(-currentHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(headingPower), 1);
        double frontLeftPower = (yPower + xPower + headingPower) / denominator;
        double backLeftPower = (yPower - xPower + headingPower) / denominator;
        double frontRightPower = (yPower - xPower - headingPower) / denominator;
        double backRightPower = (yPower + xPower - headingPower) / denominator;

        robot.drive.getFrontLeft().setPower(frontLeftPower);
        robot.drive.getFrontRight().setPower(frontRightPower);
        robot.drive.getRearLeft().setPower(backLeftPower);
        robot.drive.getRearRight().setPower(backRightPower);

        // Return False to end the while when all conditions are met
        return !(x > -Constants.doubleErrorThreshold) || !(x < Constants.doubleErrorThreshold) || !(y > -Constants.doubleErrorThreshold) || !(y < Constants.doubleErrorThreshold) || !(heading > -Constants.doubleErrorThreshold) || !(heading < Constants.doubleErrorThreshold);
    }
}
//package org.firstinspires.ftc.teamcode.Old.Subsystems.LegacySubsystems;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Old.Utilities.MoreOld.Constants;
//import org.firstinspires.ftc.teamcode.Utilites.GoBildaPinpointDriver;
//
//import java.util.Locale;
//
//public class Odometry {
//    private final GoBildaPinpointDriver odo;
//    private final Compass compass;
//    // isInitialized starts out false and turns true after the first `setPosition()` or `updateWithWeight()`
//    private boolean isInitialized = false;
//
//
//    public Odometry(GoBildaPinpointDriver odometry, Compass compass) {
//        this.odo = odometry;
//        this.compass = compass;
//
//        // Sets Odometry offsets
//        this.odo.setOffsets(-173.0, 156); //measured in mm
//        this.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        this.odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        this.compass.resetYaw();
//        this.odo.resetPosAndIMU();
//    }
//
//    /**
//     * Updates the robot's position on the field.
//     * <p>
//     * This method takes x and y coordinates as input and updates
//     * the odometry with the new position and the current heading
//     * from the IMU. It also marks the odometry as initialized if
//     * it wasn't already.
//     *
//     * @param x The x-coordinate of the robot's position in inches.
//     * @param y The y-coordinate of the robot's position in inches.
//     */
//    public void setPosition(double x, double y) {
//        odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, compass.getHeading()));
//        if (!isInitialized) {
//            isInitialized = true;
//        }
//    }
//
//    /**
//     * Updates the robot's position on the field.
//     * <p>
//     * This method takes the x coordinate, y coordinate, and heading
//     * as input and updates the odometry with the new position heading.
//     * It also marks the odometry as initialized if it wasn't already.
//     *
//     * @param x       The x-coordinate of the robot's position in inches.
//     * @param y       The y-coordinate of the robot's position in inches.
//     * @param heading The heading of the robot in degrees.
//     */
//    public void setPosition(double x, double y, double heading) {
//        odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
//        if (!isInitialized) {
//            isInitialized = true;
//        }
//    }
//
//    public void update() {
//        odo.update();
//    }
//
//    /**
//     * Updates the robot's position using a weighted average between the current odometry position and new sensor data.
//     * <p>
//     * This method calculates a new position by blending the current odometry position with new position data (x, y)
//     * using a weight defined by {@link Constants#odometryWeight}. This allows for smoother and more accurate position
//     * tracking by incorporating both odometry and external sensor information.
//     *
//     * @param x The x-coordinate of the new position data (in inches).
//     * @param y The y-coordinate of the new position data (in inches).
//     */
//    public void updateWithWeight(double x, double y) {
//        double currentX = odo.getPosX();
//        double currentY = odo.getPosY();
//        double currentHeading = compass.getHeading();
//
//        odo.setPosition(new Pose2D(DistanceUnit.INCH, (currentX * (1 - Constants.odometryWeight)) + (x * Constants.odometryWeight), (currentY * (1 - Constants.odometryWeight)) + (y * Constants.odometryWeight), AngleUnit.DEGREES, currentHeading));
//    }
//
//    /**
//     * Updates the robot's position using a weighted average between the current odometry position and new sensor data.
//     * <p>
//     * This method calculates a new position by blending the current odometry position with new position data (x, y, and heading)
//     * using a weight defined by {@link Constants#odometryWeight}. This allows for smoother and more accurate position
//     * tracking by incorporating both odometry and external sensor information.
//     *
//     * @param x       The x-coordinate of the new position data (in inches).
//     * @param y       The y-coordinate of the new position data (in inches).
//     * @param heading The heading of the new position data (in degrees).
//     */
//    public void updateWithWeight(double x, double y, double heading) {
//        double currentX = odo.getPosX();
//        double currentY = odo.getPosY();
//        double currentHeading = compass.getHeading();
//
//        odo.setPosition(new Pose2D(DistanceUnit.INCH, (currentX * (1 - Constants.odometryWeight)) + (x * Constants.odometryWeight), (currentY * (1 - Constants.odometryWeight)) + (y * Constants.odometryWeight), AngleUnit.DEGREES, (currentHeading * (1 - Constants.odometryWeight)) + (heading * Constants.odometryWeight)));
//    }
//
//    /**
//     * Gets the current position of the robot.
//     * <p>
//     * This method retrieves the robot's current position from the odometry system.
//     * The position is represented as a Pose2D object, which includes the x and y coordinates
//     * and the heading (rotation) of the robot.
//     *
//     * @return The current position of the robot as a Pose2D object.
//     */
//    public Pose2D getPosition() {
//        return odo.getPosition();
//    }
//
//    /**
//     * Gets the current telemetry data from the robot's odometry.
//     *
//     * @return A formatted string containing the robot's X position, Y position, and heading.
//     * Values are in inches and degrees respectively.
//     */
//    public String getTelemetry() {
//        Pose2D position = odo.getPosition();
//        return String.format(Locale.getDefault(), "X: %f, Y: %f, Heading: %f", position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH), position.getHeading(AngleUnit.DEGREES));
//    }
//
//    /**
//     * Gets the current X telemetry data from the robot's odometry.
//     *
//     * @return The robot's X telemetry values
//     */
//    public double rawXTelemetry() {
//        Pose2D position = odo.getPosition();
//        return position.getX(DistanceUnit.INCH);
//    }
//
//    /**
//     * Gets the current Y telemetry data from the robot's odometry.
//     *
//     * @return The robot's Y telemetry values
//     */
//    public double rawYTelemetry() {
//        Pose2D position = odo.getPosition();
//        return position.getY(DistanceUnit.INCH);
//    }
//
//    /**
//     * Gets the current heading telemetry data from the robot's odometry.
//     *
//     * @return The robot's Heading telemetry values
//     */
//    public double rawHeadingTelemetry() {
//        Pose2D position = odo.getPosition();
//        return position.getHeading(AngleUnit.DEGREES);
//    }
//
//    public double XOffset() {
//        return odo.getXOffset();
//    }
//
//    public double YOffset() {
//        return odo.getYOffset();
//    }
//}

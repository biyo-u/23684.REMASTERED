//package org.firstinspires.ftc.teamcode.Old.Subsystems.LegacySubsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class OldDrive {
//    private final DcMotor frontLeft;
//    private final DcMotor frontRight;
//    private final DcMotor rearLeft;
//    private final DcMotor rearRight;
//    private final Compass compass;
//    private double power;
//
//    public OldDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight, Compass compass) {
//        this.frontLeft = frontLeft;
//        this.frontRight = frontRight;
//        this.rearLeft = rearLeft;
//        this.rearRight = rearRight;
//        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        this.frontRight.setDirection(DcMotor.Direction.FORWARD);
//        this.rearLeft.setDirection(DcMotor.Direction.REVERSE);
//        this.rearRight.setDirection(DcMotor.Direction.FORWARD);
//        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.compass = compass;
//        this.power = 0.5;
//    }
//
//    public void driveMecanumRobotCentric(double left_stick_y, double left_stick_x, double right_stick_x) {
//        double y = -left_stick_y; // Remember, Y stick value is reversed
//        double x = left_stick_x; // Counteract imperfect strafing
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(right_stick_x), 1);
//        double frontLeftPower = (y + x + right_stick_x) / denominator;
//        double backLeftPower = (y - x + right_stick_x) / denominator;
//        double frontRightPower = (y - x - right_stick_x) / denominator;
//        double backRightPower = (y + x - right_stick_x) / denominator;
//
//        frontLeft.setPower(frontLeftPower);
//        rearLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        rearRight.setPower(backRightPower);
//    }
//
//    public void driveMecanumFieldCentric(double left_stick_y, double left_stick_x, double right_stick_x, double botHeading) {
//        double y = -left_stick_y; // Remember, Y stick value is reversed
//
//        // Gets robot heading (direction it's pointing)
//        //double botHeading = compass.getHeading();
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = left_stick_x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = left_stick_x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        // TODO: Make sure this is needed and doesn't just cause problems
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(right_stick_x), 1);
//        double frontLeftPower = ((rotY + rotX + right_stick_x) / denominator) * power;
//        double backLeftPower = ((rotY - rotX + right_stick_x) / denominator) * power;
//        double frontRightPower = ((rotY - rotX - right_stick_x) / denominator) * power;
//        double backRightPower = ((rotY + rotX - right_stick_x) / denominator) * power;
//
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        rearLeft.setPower(backLeftPower);
//        rearRight.setPower(backRightPower);
//    }
//
//    /**
//     * Sets the power level of the object.
//     *
//     * @param power The new power level to be set.
//     */
//    public void setPower(double power) {
//        this.power = power;
//    }
//
//    public DcMotor getFrontLeft() {
//        return frontLeft;
//    }
//
//    public DcMotor getFrontRight() {
//        return frontRight;
//    }
//
//    public DcMotor getRearLeft() {
//        return rearLeft;
//    }
//
//    public DcMotor getRearRight() {
//        return rearRight;
//    }
//}

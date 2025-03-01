package org.firstinspires.ftc.teamcode.Old.Subsystems.LegacySubsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilites.GoBildaPinpointDriver;

import java.util.Locale;

public class Compass {
	private final IMU imu;
	private final GoBildaPinpointDriver odocomp;

	private String model;

	public Compass(IMU imu) {
		this.imu = imu;
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
				RevHubOrientationOnRobot.UsbFacingDirection.UP));
		imu.initialize(parameters);
		odocomp = null;
	}

	public double getHeading() {
		if (odocomp != null) {
			model = "Pinpoint IMU";
			odocomp.update();
			return odocomp.getHeading();
		} else {
			model = "REV Hub IMU";
			return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
		}
	}
	public void resetYaw() {
		if (imu != null) {
			imu.resetYaw();
		} else {
			odocomp.update();
			Pose2D prevLocation = odocomp.getPosition();
			odocomp.resetPosAndIMU();
			odocomp.setPosition(new Pose2D(DistanceUnit.INCH, prevLocation.getX(DistanceUnit.INCH), prevLocation.getY(DistanceUnit.INCH),AngleUnit.DEGREES,0));
		}
	}

	public Compass(GoBildaPinpointDriver goBildaPinpointDriver) {
		this.odocomp = goBildaPinpointDriver;
		imu = null;
	}

	public String getTelemetry() {
		if (imu != null) {
			return String.format(Locale.getDefault(), """
                    Robot Yaw: %f
                    Robot Pitch: %f
                    Robot Roll: %f
                    Model Used: %s""", imu.getRobotYawPitchRollAngles().getYaw(), imu.getRobotYawPitchRollAngles().getPitch(), imu.getRobotYawPitchRollAngles().getRoll(), model);
		} else {
			odocomp.update();
			return String.format(Locale.getDefault(), """
                Robot Yaw: %f
                Model Used: %s""", odocomp.getHeading(), model);
		}
	}
}

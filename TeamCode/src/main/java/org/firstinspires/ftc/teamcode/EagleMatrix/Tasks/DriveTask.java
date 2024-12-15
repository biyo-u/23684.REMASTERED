package org.firstinspires.ftc.teamcode.EagleMatrix.Tasks;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Task;

public class DriveTask extends Task {
	private GoBildaPinpointDriver odometry;

	Pose2D targetPosition;
	Pose2D currentPosition;

	public DriveTask(Pose2D targetPosition){
		this.targetPosition = targetPosition;
	}

	@Override
	public boolean run(){
		return false;
	}
}

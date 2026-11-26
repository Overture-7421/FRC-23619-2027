package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utils.PoseStorage;

public class Chassis extends SubsystemBase {
	private final Follower follower;
	private Pose startPose;
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	public Chassis(Follower follower) {
		this.follower = follower;
	}

	@Override
	public void periodic() {
		follower.update();

		Pose pose = follower.pose();
		if (startPose != null) {
			if (Math.hypot(pose.x() - startPose.x(), pose.y() - startPose.y()) > 0.5)
				follower.setPose(startPose);
			startPose = null;
		}
		// telemetry.addData("Robot X", pose.x());
		// telemetry.addData("Robot Y", pose.y());
		// telemetry.addData("Robot Heading deg", Math.toDegrees(pose.heading()));
	}

	public Follower getFollower() {
		return follower;
	}

	public void initPose(boolean isAuto, Pose starting) {
		startPose = PoseStorage.currentPose != null && !isAuto ? PoseStorage.currentPose : starting;
		follower.setPose(startPose);
	}

	public void setLastPose() {
		PoseStorage.currentPose = follower.pose();
	}
}

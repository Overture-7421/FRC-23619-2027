package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utils.PoseStorage;

public class Chassis extends SubsystemBase {
	private final Follower follower;
	private Pose startPose;
	private int startTries = 0;
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	public Chassis(Follower follower) {
		this.follower = follower;
	}

	@Override
	public void periodic() {
		follower.update();
		if (startPose != null && (startPoseHolds("run") || ++startTries > 25))
			startPose = null;
		// telemetry.addData("Robot X", follower.pose().x());
		// telemetry.addData("Robot Y", follower.pose().y());
		// telemetry.addData("Robot Heading deg", Math.toDegrees(follower.pose().heading()));
	}

	public Follower getFollower() {
		return follower;
	}

	public void initPose(boolean isAuto, Pose starting) {
		startPose = PoseStorage.currentPose != null && !isAuto ? PoseStorage.currentPose : starting;
		follower.setPose(startPose);
		for (int i = 0; i < 5; i++) {
			SystemClock.sleep(20);
			follower.localizer.update();
			if (startPoseHolds("init"))
				break;
		}
	}

	private boolean startPoseHolds(String when) {
		Pose pose = follower.pose();
		boolean holds = Math.hypot(pose.x() - startPose.x(), pose.y() - startPose.y()) < 0.5;
		if (!holds)
			follower.setPose(startPose);
		return holds;
	}

	public void setLastPose() {
		PoseStorage.currentPose = follower.pose();
	}
}

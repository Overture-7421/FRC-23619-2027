package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utils.PoseStorage;

public class Chassis extends SubsystemBase {
	private final Follower follower;

	public Chassis(Follower follower) {
		this.follower = follower;
	}

	@Override
	public void periodic() {
		follower.update();
	}

	public Follower getFollower() {
		return follower;
	}

	public void initPose(boolean isAuto, Pose starting) {
		if (PoseStorage.currentPose != null && !isAuto) {
			follower.setPose(PoseStorage.currentPose);
		} else {
			follower.setPose(starting);
		}
	}

	public void setLastPose() {
		PoseStorage.currentPose = follower.pose();
	}
}

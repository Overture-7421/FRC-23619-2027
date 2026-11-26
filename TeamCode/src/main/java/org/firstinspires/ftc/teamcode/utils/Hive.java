package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.MyRobot.Alliance;

public class Hive {

	public enum Tilt {
		AudienceUp,
		ScoringUp
	}

	public static final double RED_X = 59.26;
	public static final double BLUE_X = 84.74;
	public static final double AUDIENCE_CELL_Y = 57.67;
	public static final double SCORING_CELL_Y = 86.33;

	public static final double TAG_ROW = 12.36;
	public static final double TAG_TILT_SHIFT = 0.75;
	public static final double[] TAG_OFFSETS = { -6.50, -2.75, 2.75, 6.50 };

	public static final double CELL_HEIGHT = 58.04;
	public static final double UP_TAG_HEIGHT = 49.79;
	public static final double DOWN_TAG_HEIGHT = 35.51;

	public static Pose getTarget(Alliance alliance, Tilt tilt) {
		double x = alliance == Alliance.Red ? RED_X : BLUE_X;
		double y = tilt == Tilt.AudienceUp ? AUDIENCE_CELL_Y : SCORING_CELL_Y;
		return new Pose(x, y);
	}

	public static Tilt getStartTilt(Alliance alliance) {
		return alliance == Alliance.Red ? Tilt.AudienceUp : Tilt.ScoringUp;
	}

	public static boolean isHiveTag(int id) {
		return id >= 30 && id < 46;
	}

	public static boolean isOwnTag(Alliance alliance, int id) {
		int first = alliance == Alliance.Red ? 30 : 38;
		return id >= first && id < first + 8;
	}

	public static boolean isAudienceTag(int id) {
		return id >= 34 && id <= 41;
	}

	public static Pose getTagPose(int id, Tilt tilt) {
		int side = isAudienceTag(id) ? 1 : -1;
		double x = (id < 38 ? RED_X : BLUE_X) + side * TAG_OFFSETS[(id - 30) % 4];
		double y = 72 - side * TAG_ROW - (tilt == Tilt.AudienceUp ? TAG_TILT_SHIFT : -TAG_TILT_SHIFT);
		return new Pose(x, y);
	}
}

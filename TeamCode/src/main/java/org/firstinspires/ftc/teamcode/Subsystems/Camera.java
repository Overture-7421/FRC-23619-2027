package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.function.Supplier;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.MyRobot.Alliance;
import org.firstinspires.ftc.teamcode.utils.Hive;
import org.firstinspires.ftc.teamcode.utils.TurretMath;

@Configurable
public class Camera extends SubsystemBase {
	public static double cameraForwardIn = 5.361;
	public static double cameraHeightIn = 16.785;
	public static double cameraPitchDeg = 45.0;
	public static double maxHeightErrorIn = 5.0;
	public static int framesToSwitch = 6;

	public static boolean useTrim = true;
	public static double trimBlend = 0.1;
	public static double maxTrimIn = 12.0;
	public static int framesToAcquire = 3;
	public static double acquireAgreementIn = 2.0;
	public static double maxHeadingErrorDeg = 10.0;
	public static double headingAgreementDeg = 3.0;
	public static int minHeadingTags = 3;

	private final Limelight3A limelight;
	private final Supplier<Pose> turretPose;
	private final Alliance alliance;
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private Hive.Tilt tilt;
	private int votes = 0;
	private int tagsSeen = 0;
	private double lastTagHeight = 0.0;
	private double lastHeightSum = 0.0;
	private double trimX = 0.0;
	private double trimY = 0.0;
	private boolean trimActive = true;
	private int largeFrames = 0;
	private double lastLargeX = 0.0;
	private double lastLargeY = 0.0;
	private double headingErrorDeg = 0.0;
	private int headingFrames = 0;
	private Pose pivotFix = new Pose(0, 0, 0);
	private final double[][] seen = new double[8][4];

	public Camera(HardwareMap hardwareMap, Alliance alliance, Supplier<Pose> turretPose) {
		this.alliance = alliance;
		this.turretPose = turretPose;
		tilt = Hive.getStartTilt(alliance);
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(100);
		limelight.pipelineSwitch(0);
		limelight.start();
	}

	@Override
	public void periodic() {
		readTags();

		telemetry.addData("Camera Tilt", tilt);
		telemetry.addData("Camera Tags", tagsSeen);
		telemetry.addData("Camera TagHeight", lastTagHeight);
		telemetry.addData("Camera TrimX", trimX);
		telemetry.addData("Camera TrimY", trimY);
		telemetry.addData("Camera Acquiring", largeFrames >= framesToAcquire);
		telemetry.addData("Camera HeadingError deg", headingErrorDeg);
	}

	public Hive.Tilt getTilt() {
		return tilt;
	}

	public double getTrimX() {
		return trimX;
	}

	public double getTrimY() {
		return trimY;
	}

	public void setTrimActive(boolean trimActive) {
		this.trimActive = trimActive;
	}

	public void resetTrim() {
		trimX = 0.0;
		trimY = 0.0;
		headingFrames = 0;
	}

	public boolean hasTags() {
		return tagsSeen > 0;
	}

	public boolean hasHeadingFix() {
		return headingFrames >= framesToAcquire && Math.abs(headingErrorDeg) > maxHeadingErrorDeg;
	}

	public double getHeadingError() {
		return Math.toRadians(headingErrorDeg);
	}

	public Pose getPivotFix() {
		return pivotFix;
	}

	private void readTags() {
		tagsSeen = 0;
		LLResult result = limelight.getLatestResult();
		if (result == null || !result.isValid())
			return;

		double pitch = Math.toRadians(cameraPitchDeg);
		Pose pivot = turretPose.get();
		double direction = pivot.heading();
		double heightSum = 0.0;
		double errorX = 0.0;
		double errorY = 0.0;
		int audienceUp = 0;
		int scoringUp = 0;

		for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
			int id = tag.getFiducialId();
			if (!Hive.isOwnTag(alliance, id))
				continue;

			Position position = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
			double height = cameraHeightIn + position.z * Math.sin(pitch) - position.y * Math.cos(pitch);
			double forward = cameraForwardIn + position.z * Math.cos(pitch) + position.y * Math.sin(pitch);
			double left = -position.x;
			heightSum += height;
			lastTagHeight = height;

			Pose tagPose = Hive.getTagPose(id, tilt);
			if (tagsSeen < seen.length) {
				seen[tagsSeen][0] = forward;
				seen[tagsSeen][1] = left;
				seen[tagsSeen][2] = tagPose.x();
				seen[tagsSeen][3] = tagPose.y();
			}
			tagsSeen++;
			errorX += pivot.x() + forward * Math.cos(direction) - left * Math.sin(direction) - tagPose.x();
			errorY += pivot.y() + forward * Math.sin(direction) + left * Math.cos(direction) - tagPose.y();

			boolean isUp;
			if (Math.abs(height - Hive.UP_TAG_HEIGHT) < maxHeightErrorIn)
				isUp = true;
			else if (Math.abs(height - Hive.DOWN_TAG_HEIGHT) < maxHeightErrorIn)
				isUp = false;
			else
				continue;

			if (isUp == Hive.isAudienceTag(id))
				audienceUp++;
			else
				scoringUp++;
		}

		if (tagsSeen == 0 || heightSum == lastHeightSum)
			return;
		lastHeightSum = heightSum;

		updateHeading(Math.toDegrees(direction));
		if (Math.abs(headingErrorDeg) <= maxHeadingErrorDeg)
			updateTrim(errorX / tagsSeen, errorY / tagsSeen);
		updateTilt(audienceUp, scoringUp);
	}

	private void updateHeading(double believedDeg) {
		int count = Math.min(tagsSeen, seen.length);
		if (count < minHeadingTags) {
			headingFrames = 0;
			headingErrorDeg = 0.0;
			return;
		}

		double meanForward = 0, meanLeft = 0, meanX = 0, meanY = 0;
		for (int i = 0; i < count; i++) {
			meanForward += seen[i][0] / count;
			meanLeft += seen[i][1] / count;
			meanX += seen[i][2] / count;
			meanY += seen[i][3] / count;
		}
		double cross = 0, dot = 0;
		for (int i = 0; i < count; i++) {
			double forward = seen[i][0] - meanForward, left = seen[i][1] - meanLeft;
			double x = seen[i][2] - meanX, y = seen[i][3] - meanY;
			cross += forward * y - left * x;
			dot += forward * x + left * y;
		}

		double seenDirection = Math.atan2(cross, dot);
		double cos = Math.cos(seenDirection), sin = Math.sin(seenDirection);
		pivotFix = new Pose(meanX - meanForward * cos + meanLeft * sin, meanY - meanForward * sin - meanLeft * cos,
				seenDirection);

		double error = TurretMath.wrap180(believedDeg - Math.toDegrees(seenDirection));
		headingFrames = Math.abs(TurretMath.wrap180(error - headingErrorDeg)) < headingAgreementDeg ? headingFrames + 1 : 1;
		headingErrorDeg = error;
	}

	private void updateTrim(double errorX, double errorY) {
		if (!useTrim) {
			trimX = 0.0;
			trimY = 0.0;
			return;
		}

		if (!trimActive)
			return;

		if (Math.hypot(errorX, errorY) > maxTrimIn) {
			boolean agrees = Math.hypot(errorX - lastLargeX, errorY - lastLargeY) < acquireAgreementIn;
			largeFrames = agrees ? largeFrames + 1 : 1;
			lastLargeX = errorX;
			lastLargeY = errorY;
			if (largeFrames >= framesToAcquire) {
				trimX = errorX;
				trimY = errorY;
			}
			return;
		}

		largeFrames = 0;
		trimX += trimBlend * (errorX - trimX);
		trimY += trimBlend * (errorY - trimY);
	}

	private void updateTilt(int audienceUp, int scoringUp) {
		if (audienceUp == scoringUp)
			return;

		Hive.Tilt seen = audienceUp > scoringUp ? Hive.Tilt.AudienceUp : Hive.Tilt.ScoringUp;
		if (seen == tilt) {
			votes = 0;
			return;
		}

		votes++;
		if (votes < framesToSwitch)
			return;

		votes = 0;
		tilt = seen;
	}
}

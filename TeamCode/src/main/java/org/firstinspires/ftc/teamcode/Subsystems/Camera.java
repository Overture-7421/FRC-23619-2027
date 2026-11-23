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

@Configurable
public class Camera extends SubsystemBase {
	public static double cameraForwardIn = 6.77;
	public static double cameraHeightIn = 16.785;
	public static double cameraPitchDeg = 45.0;
	public static double maxHeightErrorIn = 5.0;
	public static int framesToSwitch = 6;

	public static boolean useTrim = true;
	public static double trimBlend = 0.1;
	public static double maxTrimIn = 12.0;
	public static int framesToAcquire = 3;
	public static double acquireAgreementIn = 2.0;

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
		// telemetry.addData("Camera TagHeight", lastTagHeight);
		// telemetry.addData("Camera TrimX", trimX);
		// telemetry.addData("Camera TrimY", trimY);
		// telemetry.addData("Camera Acquiring", largeFrames >= framesToAcquire);
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
	}

	public boolean hasTags() {
		return tagsSeen > 0;
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
			tagsSeen++;

			Pose tagPose = Hive.getTagPose(id, tilt);
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

		updateTrim(errorX / tagsSeen, errorY / tagsSeen);
		updateTilt(audienceUp, scoringUp);
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

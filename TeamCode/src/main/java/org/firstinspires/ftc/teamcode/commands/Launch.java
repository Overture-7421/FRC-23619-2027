package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.BooleanSupplier;

import org.firstinspires.ftc.teamcode.MyRobot.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Hive;
import org.firstinspires.ftc.teamcode.utils.LaunchConstants;
import org.firstinspires.ftc.teamcode.utils.TargetingWhileMoving;

@Configurable
public class Launch extends CommandBase {
	public static boolean keepShooterOn = false;
	public static boolean shootOnTheMove = true;
	public static boolean useManual = false;
	public static double manualSpeed = 0.0;
	public static double manualHood = 0.0;
	public static double maxFeedErrorDeg = 5.0;
	public static double maxApproachAngleDeg = 80.0;
	public static double maxTrimSpeed = 10.0;
	public static double maxTrimOmega = 0.5;
	public static boolean correctPose = true;
	public static double poseCorrectionTime = 0.5;
	public static double minPoseCorrectionIn = 1.0;
	public static double maxTurretRateDeg = 30.0;
	public static int calmLoopsToFix = 5;

	private final Turret turret;
	private final Shooter shooter;
	private final Hood hood;
	private final Camera camera;
	private final Follower follower;
	private final Alliance alliance;
	private final BooleanSupplier shootRequested;
	private final TargetingWhileMoving targetWhileMoving = new TargetingWhileMoving(
			LaunchConstants.DistanceToTimeOfFlight);
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private boolean feeding = false;
	private double lastTurretAngle = 0.0;
	private long lastTime = 0;
	private int calmLoops = 0;
	private boolean facingOpening = false;
	private long steadySince = 0;
	private int poseCorrections = 0;

	public Launch(Turret turret, Shooter shooter, Hood hood, Camera camera, Chassis chassis, Alliance alliance,
			BooleanSupplier shootRequested) {
		this.turret = turret;
		this.shooter = shooter;
		this.hood = hood;
		this.camera = camera;
		this.follower = chassis.getFollower();
		this.alliance = alliance;
		this.shootRequested = shootRequested;
		addRequirements(turret, shooter, hood);
	}

	@Override
	public void execute() {
		Pose robot = follower.pose();
		Velocity velocity = follower.velocity();
		Pose turretPose = turret.getTurretPose(robot);
		double vx = velocity.vx - velocity.omega * (turretPose.y() - robot.y());
		double vy = velocity.vy + velocity.omega * (turretPose.x() - robot.x());

		Pose cell = Hive.getTarget(alliance, camera.getTilt());
		Pose target = new Pose(cell.x() + camera.getTrimX(), cell.y() + camera.getTrimY());
		targetWhileMoving.setTargetLocation(target);

		double outward = camera.getTilt() == Hive.Tilt.AudienceUp ? target.y() - turretPose.y()
				: turretPose.y() - target.y();
		double approach = Math.toDegrees(Math.atan2(Math.abs(turretPose.x() - target.x()), outward));
		facingOpening = approach < maxApproachAngleDeg;
		Pose movingTarget = shootOnTheMove ? targetWhileMoving.getMovingTarget(turretPose, vx, vy) : target;

		double dx = movingTarget.x() - turretPose.x();
		double dy = movingTarget.y() - turretPose.y();
		double distance = Math.hypot(dx, dy);

		turret.aimAtFieldPosition(robot, movingTarget);
		turret.setAimRate((dy * vx - dx * vy) / Math.max(dx * dx + dy * dy, 1e-6) - velocity.omega);
		boolean shooting = shootRequested.getAsBoolean();
		double speed = useManual ? manualSpeed : LaunchConstants.DistanceToShooter.get(distance);
		shooter.setSpeed(keepShooterOn || shooting ? speed : 0);
		hood.setPosition(useManual ? manualHood : LaunchConstants.DistanceToHood.get(distance));

		long now = System.nanoTime();
		double turretRate = lastTime == 0 ? 0.0 : (turret.getAngle() - lastTurretAngle) / ((now - lastTime) / 1e9);
		lastTurretAngle = turret.getAngle();
		lastTime = now;

		boolean robotSlow = Math.hypot(velocity.vx, velocity.vy) < maxTrimSpeed
				&& Math.abs(velocity.omega) < maxTrimOmega;
		boolean steady = turret.isAtPosition() && robotSlow;
		camera.setTrimActive(steady);

		calmLoops = robotSlow && Math.abs(turretRate) < maxTurretRateDeg ? calmLoops + 1 : 0;
		if (correctPose && calmLoops >= calmLoopsToFix && camera.hasHeadingFix()) {
			follower.setPose(turret.getRobotPose(camera.getPivotFix(), robot.heading() - camera.getHeadingError()));
			camera.resetTrim();
			steadySince = 0;
			poseCorrections++;
		}

		if (!steady || !camera.hasTags()) {
			steadySince = 0;
		} else if (steadySince == 0) {
			steadySince = System.nanoTime();
		} else if (correctPose && System.nanoTime() - steadySince > poseCorrectionTime * 1e9
				&& Math.hypot(camera.getTrimX(), camera.getTrimY()) > minPoseCorrectionIn) {
			follower.setPose(new Pose(robot.x() - camera.getTrimX(), robot.y() - camera.getTrimY(), robot.heading()));
			camera.resetTrim();
			steadySince = 0;
			poseCorrections++;
		}

		if (!shooting)
			feeding = false;
		else if (isReady())
			feeding = true;

		telemetry.addData("Launch Distance", distance);
		// telemetry.addData("Launch MovingTarget", movingTarget);
		// telemetry.addData("Launch Approach deg", approach);
		telemetry.addData("Launch PoseCorrections", poseCorrections);
		telemetry.addData("Launch Ready", isReady());
	}

	@Override
	public void end(boolean interrupted) {
		feeding = false;
		shooter.setSpeed(0);
	}

	public boolean canFeed() {
		return feeding && facingOpening && Math.abs(turret.getTargetAngle() - turret.getAngle()) < maxFeedErrorDeg;
	}

	public boolean isReady() {
		return facingOpening && turret.isAtPosition() && shooter.isAtSpeed();
	}
}

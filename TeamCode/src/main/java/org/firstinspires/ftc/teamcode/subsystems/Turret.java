package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.utils.TurretMath;

@Configurable
public class Turret extends SubsystemBase {
	public static final String OCTOQUAD_NAME = "octoquad";
	public static final String SERVO_NAME = "turret";
	public static final int ENCODER1_CHANNEL = 4;
	public static final int ENCODER2_CHANNEL = 5;

	public static int encoder1MinUs = 1;
	public static int encoder1MaxUs = 1024;
	public static int encoder2MinUs = 1;
	public static int encoder2MaxUs = 1024;
	public static boolean encoder1Inverted = true;
	public static boolean encoder2Inverted = true;
	public static double encoder1OffsetTurns = 0.20996;
	public static double encoder2OffsetTurns = 0.47461;
	public static boolean servoInverted = false;
	public static boolean useAxonPwmRange = true;
	public static int servoFrameUs = 5000;

	public static double kP = 0.0095;
	public static double kD = 0.0015;
	public static double kS = 0.13;
	public static double kV = 0.07;
	public static double aimLeadSec = 0.06;
	public static double trackingRateDeg = 5.0;
	public static double feedforwardWindowDeg = 60.0;
	public static double maxPower = 0.4;
	public static double rangeOfErrorDeg = 2.0;

	public static double forwardLimitDeg = 200.0;
	public static double reverseLimitDeg = -200.0;
	public static double hardLimitMarginDeg = 15.0;
	public static double turretOffsetX = 3.522;
	public static double turretOffsetY = 0.0;
	public static double aimOffsetDeg = 0.0;

	public static double maxEncoder2ResidualDeg = 20.0;
	public static int pulseSlackUs = 25;
	public static int maxBadReads = 5;

	private final OctoQuad octoQuad;
	private final CRServoEx torreta;
	private final PIDController controller = new PIDController(kP, 0.0, kD);
	private final OctoQuad.EncoderDataBlock encoderData = new OctoQuad.EncoderDataBlock();
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private double aimRate = 0.0;
	private double lastPower = 0.0;
	private boolean active = false;
	private double turretAngle = 0.0;
	private double targetAngle = 0.0;
	private double encoder1Deg, encoder2Deg, encoder2Residual;
	private int badReads = 1000;
	private String lastFault = "no read yet";

	public Turret(HardwareMap hardwareMap) {
		octoQuad = hardwareMap.get(OctoQuad.class, OCTOQUAD_NAME);
		configureOctoQuad(octoQuad);

		torreta = new CRServoEx(hardwareMap, SERVO_NAME);
		if (useAxonPwmRange) {
			torreta.setPwm(new PwmControl.PwmRange(500, 2500, servoFrameUs));
		}
		torreta.getServo().setPower(0);

		readEncoders();
		targetAngle = Range.clip(turretAngle, reverseLimitDeg, forwardLimitDeg);
	}

	public static void configureOctoQuad(OctoQuad oq) {
		oq.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);
		oq.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
		oq.setSingleChannelPulseWidthParams(ENCODER1_CHANNEL,
				new OctoQuad.ChannelPulseWidthParams(encoder1MinUs, encoder1MaxUs));
		oq.setSingleChannelPulseWidthParams(ENCODER2_CHANNEL,
				new OctoQuad.ChannelPulseWidthParams(encoder2MinUs, encoder2MaxUs));
		oq.setSingleChannelPulseWidthTracksWrap(ENCODER1_CHANNEL, false);
		oq.setSingleChannelPulseWidthTracksWrap(ENCODER2_CHANNEL, false);
		SystemClock.sleep(50);
	}

	@Override
	public void periodic() {
		boolean goodRead = readEncoders();

		double error = targetAngle - turretAngle;
		double power = 0.0;
		if (!active) {
			controller.reset();
		} else if (goodRead) {
			controller.setPID(kP, 0.0, kD);
			double aimRateDeg = Math.toDegrees(aimRate);
			boolean tracking = Math.abs(aimRateDeg) > trackingRateDeg;
			double setpoint = Range.clip(targetAngle + aimRateDeg * aimLeadSec, reverseLimitDeg, forwardLimitDeg);
			controller.setOpenF(tracking ? 0.0 : kS);
			controller.setTolerance(rangeOfErrorDeg);

			power = controller.calculate(turretAngle, setpoint);
			if (controller.atSetPoint()) {
				power = 0.0;
			}
			if (tracking && Math.abs(error) < feedforwardWindowDeg) {
				power += aimRate * kV + Math.signum(aimRate) * kS;
			}
			power = Range.clip(power, -maxPower, maxPower);

			if (turretAngle > forwardLimitDeg + hardLimitMarginDeg)
				power = Math.min(power, 0.0);
			if (turretAngle < reverseLimitDeg - hardLimitMarginDeg)
				power = Math.max(power, 0.0);
			lastPower = power;
		} else if (isHealthy()) {
			power = lastPower;
		}
		torreta.set(servoInverted ? -power : power);
		com.qualcomm.robotcore.util.RobotLog.dd("TempTurret", "loop target=%.1f a=%.2f p=%.3f ok=%b", targetAngle,
				turretAngle, power, goodRead); // TEMP log

		// telemetry.addData("Turret Angle", turretAngle);
		// telemetry.addData("Turret Target", targetAngle);
		// telemetry.addData("Turret Error", error);
		// telemetry.addData("Turret Power", power);
		// telemetry.addData("Turret AimRate deg/s", Math.toDegrees(aimRate));
		// telemetry.addData("Turret Encoder1", encoder1Deg);
		// telemetry.addData("Turret Encoder2", encoder2Deg);
		// telemetry.addData("Turret Encoder2 Residual", encoder2Residual);
		telemetry.addData("Turret Health", isHealthy() ? "OK" : "FAULT: " + lastFault);
		aimRate = 0.0;
	}

	public Command goToAngle(double angleDeg) {
		return new RunCommand(() -> setTargetAngle(angleDeg), this).interruptOn(this::isAtPosition);
	}

	public InstantCommand stop() {
		return new InstantCommand(() -> active = false, this);
	}

	public Pose getTurretPose(Pose robot) {
		double cos = Math.cos(robot.heading());
		double sin = Math.sin(robot.heading());
		return new Pose(robot.x() + turretOffsetX * cos - turretOffsetY * sin,
				robot.y() + turretOffsetX * sin + turretOffsetY * cos,
				robot.heading() + Math.toRadians(turretAngle));
	}

	public Pose getRobotPose(Pose pivot, double heading) {
		double cos = Math.cos(heading);
		double sin = Math.sin(heading);
		return new Pose(pivot.x() - turretOffsetX * cos + turretOffsetY * sin,
				pivot.y() - turretOffsetX * sin - turretOffsetY * cos, heading);
	}

	public void aimAtFieldPosition(Pose robot, Pose target) {
		Pose pivot = getTurretPose(robot);
		double aim = TurretMath.wrap180(Math.toDegrees(
				Math.atan2(target.y() - pivot.y(), target.x() - pivot.x()) - robot.heading())) + aimOffsetDeg;
		setTargetAngle(TurretMath.convertToClosestBoundedTurretAngleDegrees(
				aim, turretAngle, forwardLimitDeg, reverseLimitDeg));
	}

	public void setAimRate(double radPerSec) {
		aimRate = radPerSec;
	}

	public void setTargetAngle(double angleDeg) {
		targetAngle = Range.clip(angleDeg, reverseLimitDeg, forwardLimitDeg);
		active = true;
	}

	public double getAngle() {
		return turretAngle;
	}

	public double getTargetAngle() {
		return targetAngle;
	}

	public boolean isAtPosition() {
		return isHealthy() && Math.abs(targetAngle - turretAngle) < rangeOfErrorDeg;
	}

	public boolean isHealthy() {
		return badReads < maxBadReads;
	}

	private boolean readEncoders() {
		octoQuad.readAllEncoderData(encoderData);
		int pulse1 = encoderData.positions[ENCODER1_CHANNEL];
		int pulse2 = encoderData.positions[ENCODER2_CHANNEL];

		if (!encoderData.crcOk)
			return fault("I2C CRC error");
		if (!pulseInRange(pulse1, encoder1MinUs, encoder1MaxUs))
			return fault("encoder 1 no signal / out of range");
		if (!pulseInRange(pulse2, encoder2MinUs, encoder2MaxUs))
			return fault("encoder 2 no signal / out of range");

		encoder1Deg = TurretMath.encoderDegrees(pulse1, encoder1MinUs, encoder1MaxUs, encoder1Inverted,
				encoder1OffsetTurns);
		encoder2Deg = TurretMath.encoderDegrees(pulse2, encoder2MinUs, encoder2MaxUs, encoder2Inverted,
				encoder2OffsetTurns);

		double angle = TurretMath.calculateTurretAngleDegrees(encoder1Deg, encoder2Deg);
		encoder2Residual = TurretMath.encoder2ResidualDegrees(angle, encoder2Deg);
		if (Math.abs(encoder2Residual) > maxEncoder2ResidualDeg) {
			return fault("encoders disagree (slipped gear or bad offset?)");
		}

		turretAngle = angle;
		badReads = 0;
		return true;
	}

	private boolean fault(String why) {
		lastFault = why;
		badReads++;
		return false;
	}

	private static boolean pulseInRange(int us, int min, int max) {
		return us > 0 && us >= min - pulseSlackUs && us <= max + pulseSlackUs;
	}
}

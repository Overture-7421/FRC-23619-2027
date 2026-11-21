package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.TurretMath;

@TeleOp(name = "Turret Calibration", group = "Turret")
public class TurretCalibration extends CommandOpMode {
	private OctoQuad octoQuad;
	private CRServoEx torreta;
	private final OctoQuad.EncoderDataBlock data = new OctoQuad.EncoderDataBlock();

	private int min1, max1, min2, max2;
	private String offset1 = "(press X)", offset2 = "(press X)";

	@Override
	public void initialize() {
		super.reset();
		octoQuad = hardwareMap.get(OctoQuad.class, Turret.OCTOQUAD_NAME);
		torreta = new CRServoEx(hardwareMap, Turret.SERVO_NAME);
		Turret.configureOctoQuad(octoQuad);
		resetSeen();
	}

	@Override
	public void run() {
		double power = -0.3 * gamepad1.left_stick_x;
		torreta.set(Turret.servoInverted ? -power : power);

		octoQuad.readAllEncoderData(data);
		int p1 = data.positions[Turret.ENCODER1_CHANNEL];
		int p2 = data.positions[Turret.ENCODER2_CHANNEL];

		if (gamepad1.y)
			resetSeen();
		if (data.crcOk && p1 > 0) {
			min1 = Math.min(min1, p1);
			max1 = Math.max(max1, p1);
		}
		if (data.crcOk && p2 > 0) {
			min2 = Math.min(min2, p2);
			max2 = Math.max(max2, p2);
		}

		double raw1 = TurretMath.encoderDegrees(p1, Turret.encoder1MinUs, Turret.encoder1MaxUs, Turret.encoder1Inverted,
				0.0);
		double raw2 = TurretMath.encoderDegrees(p2, Turret.encoder2MinUs, Turret.encoder2MaxUs, Turret.encoder2Inverted,
				0.0);
		if (gamepad1.x) {
			offset1 = String.format("%.5f", -TurretMath.wrap180(raw1) / 360.0);
			offset2 = String.format("%.5f", -TurretMath.wrap180(raw2) / 360.0);
		}

		double e1 = TurretMath.encoderDegrees(p1, Turret.encoder1MinUs, Turret.encoder1MaxUs, Turret.encoder1Inverted,
				Turret.encoder1OffsetTurns);
		double e2 = TurretMath.encoderDegrees(p2, Turret.encoder2MinUs, Turret.encoder2MaxUs, Turret.encoder2Inverted,
				Turret.encoder2OffsetTurns);
		double angle = TurretMath.calculateTurretAngleDegrees(e1, e2);

		telemetry.addData("CRC ok", data.crcOk);
		telemetry.addData("Servo power", "%.2f", power);
		telemetry.addLine("--- 1: pulse range (sweep slowly) ---");
		telemetry.addData("Enc1 raw / seen min / max", "%d / %d / %d us", p1, min1, max1);
		telemetry.addData("Enc2 raw / seen min / max", "%d / %d / %d us", p2, min2, max2);
		telemetry.addLine("--- 2: both go UP when turret turns CCW ---");
		telemetry.addData("Enc1 angle", "%.1f deg", raw1);
		telemetry.addData("Enc2 angle", "%.1f deg", raw2);
		telemetry.addLine("--- 3: turret at robot front, press X ---");
		telemetry.addData("encoder1OffsetTurns", offset1);
		telemetry.addData("encoder2OffsetTurns", offset2);
		telemetry.addLine("--- 4: result with deployed values ---");
		telemetry.addData("Turret angle", "%.2f deg (valid +/-%.0f)", angle, TurretMath.UNIQUE_RANGE_DEG / 2.0);
		telemetry.addData("Encoder 2 residual", "%.1f deg (should be near 0)",
				TurretMath.encoder2ResidualDegrees(angle, e2));
		telemetry.update();
	}

	@Override
	public void end() {
		torreta.set(0);
	}

	private void resetSeen() {
		min1 = min2 = Integer.MAX_VALUE;
		max1 = max2 = Integer.MIN_VALUE;
	}
}

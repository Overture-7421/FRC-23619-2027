package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

@Configurable
public class Shooter extends SubsystemBase {
	public static PIDFController controlSystem = new PIDFController(0.004, 0.0, 0.0, 0.00046);
	public static double TOL = 45;
	public static double nominalVoltage = 11.8;
	private static final long STABLE_NS = 120_000_000;

	private final MotorGroup shooterMotors;
	private final VoltageSensor battery;
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private long stableSince = 0;

	public Shooter(HardwareMap hardwareMap) {
		shooterMotors = new MotorGroup(new MotorEx(hardwareMap, "shooterLeft"),
				new MotorEx(hardwareMap, "shooterRight"));
		shooterMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
		battery = hardwareMap.voltageSensor.iterator().next();
		controlSystem.setSetPoint(0);
	}

	@Override
	public void periodic() {
		double power = controlSystem.calculate(shooterMotors.getVelocity()) * nominalVoltage
				/ Math.max(battery.getVoltage(), 8.0);
		shooterMotors.set(controlSystem.getSetPoint() > 0 ? Math.max(power, 0.0) : 0.0);

		telemetry.addData("Shooter Speed", shooterMotors.getVelocity());
		telemetry.addData("Shooter Goal", controlSystem.getSetPoint());
		telemetry.addData("Shooter AtSpeed", isAtSpeed());
	}

	public void setSpeed(double speed) {
		controlSystem.setSetPoint(speed);
	}

	public boolean isAtSpeed() {
		double error = Math.abs(shooterMotors.getVelocity() - controlSystem.getSetPoint());

		if (error < TOL && controlSystem.getSetPoint() > 0) {
			if (stableSince == 0) {
				stableSince = System.nanoTime();
			}
			return System.nanoTime() - stableSince > STABLE_NS;
		}

		stableSince = 0;
		return false;
	}

	public InstantCommand stopShooter() {
		return new InstantCommand(() -> setSpeed(0), this);
	}
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public class Shooter extends SubsystemBase {
	private final MotorGroup shooterMotors;
	public static PIDFController controlSystem = new PIDFController(0.00000, 0.0, 0.0, 0.000);

	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	final int targetSpeed = 0;
	int TOL = 20;

	public Shooter(HardwareMap hardwareMap) {
		shooterMotors = new MotorGroup(new MotorEx(hardwareMap, "shooterLeft"),
				new MotorEx(hardwareMap, "shooterRight"));
	}

	@Override
	public void periodic() {
		shooterMotors.set(controlSystem.calculate(shooterMotors.getVelocity()));
		telemetry.addData("Ticks Speed", shooterMotors.getVelocity());
		telemetry.addData("Goal Speed", targetSpeed);
		telemetry.update();
	}

	public boolean isAtSpeed(int target) {
		double error = Math.abs(shooterMotors.getVelocity() - target);

		if (error < TOL) {
			return true;
		}

		return false;
	}

}

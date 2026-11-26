package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Tuning", group = "Turret")
public class TurretTuning extends CommandOpMode {
	private Turret turret;
	private final ElapsedTime loopTimer = new ElapsedTime();
	private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

	@Override
	public void initialize() {
		super.reset();
		GamepadEx driver = new GamepadEx(gamepad1);
		turret = new Turret(hardwareMap);

		driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(turret.goToAngle(0));
		driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(turret.goToAngle(90));
		driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(turret.goToAngle(-90));
		driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(turret.goToAngle(180));
		driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(turret.stop());
		// TEMP: 10 degree nudges
		driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
				new com.seattlesolvers.solverslib.command.InstantCommand(
						() -> turret.setTargetAngle(turret.getTargetAngle() + 10)));
		driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
				new com.seattlesolvers.solverslib.command.InstantCommand(
						() -> turret.setTargetAngle(turret.getTargetAngle() - 10)));
	}

	@Override
	public void initialize_loop() {
		turret.periodic();
		telemetry.addData("Turret angle", "%.2f deg", turret.getAngle());
		telemetry.addData("Healthy", turret.isHealthy());
		telemetry.update();
	}

	@Override
	public void run() {
		super.run();
		com.qualcomm.robotcore.util.RobotLog.dd("TempTurret", "bat=%.2f",
				hardwareMap.voltageSensor.iterator().next().getVoltage()); // TEMP log

		telemetry.addData("Turret angle", "%.2f deg", turret.getAngle());
		telemetry.addData("Target", "%.2f deg", turret.getTargetAngle());
		telemetry.addData("At position", turret.isAtPosition());
		telemetry.addData("Healthy", turret.isHealthy());
		telemetry.addData("Loop", "%.1f ms", loopTimer.milliseconds());
		loopTimer.reset();
		telemetry.update();
		panels.update();
	}
}

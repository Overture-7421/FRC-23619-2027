package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

public class TeleopMode extends CommandOpMode {

	public MyRobot robot;
	public GamepadEx driver;
	public MyRobot.Alliance color;

	public TeleopMode(MyRobot.Alliance allianceColor) {
		color = allianceColor;
	}

	@Override
	public void initialize() {
		super.reset();
		driver = new GamepadEx(gamepad1);
		robot = new MyRobot(color, hardwareMap, driver);
		robot.initTeleop();
	}

	@Override
	public void run() {
		robot.run();
		PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
	}

	@Override
	public void end() {
		robot.onEnd();
	}

}

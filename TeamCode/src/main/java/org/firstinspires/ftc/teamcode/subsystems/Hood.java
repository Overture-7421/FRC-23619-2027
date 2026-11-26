package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Hood extends SubsystemBase {
	private final ServoEx hood;
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private double position = 1.0;

	public Hood(HardwareMap hardwareMap) {
		hood = new ServoEx(hardwareMap, "hood");
	}

	@Override
	public void periodic() {
		// telemetry.addData("Hood Position", position);
	}

	public void setPosition(double position) {
		this.position = Math.max(-1.0, Math.min(1.0, position));
		hood.set((this.position + 1.0) / 2.0);
	}

	public double getPosition() {
		return position;
	}
}

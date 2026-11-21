package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Arm extends SubsystemBase {
	private final ServoEx servoLeft;
	private final ServoEx servoRight;

	public Arm(HardwareMap hardwareMap) {
		servoLeft = new ServoEx(hardwareMap, "colibriLeft");
		servoRight = new ServoEx(hardwareMap, "colibriRight");
	}

	public Command lower() {
		return new InstantCommand(() -> {
			servoLeft.set(1);
			servoRight.set(1);
		});
	}

	public Command rise() {
		return new InstantCommand(() -> {
			servoLeft.set(0);
			servoRight.set(0);
		});
	}
}

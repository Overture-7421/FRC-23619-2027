package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class Arm extends SubsystemBase {
	public static double closedPosition = 0.055;
	public static double openPosition = 0.0;
	public static boolean rightInverted = false;

	private final ServoEx servoLeft;
	private final ServoEx servoRight;

	public Arm(HardwareMap hardwareMap) {
		servoLeft = new ServoEx(hardwareMap, "colibriLeft");
		servoRight = new ServoEx(hardwareMap, "colibriRight");
		servoRight.setInverted(rightInverted);
		setPosition(closedPosition);
	}

	public Command open() {
		return new InstantCommand(() -> setPosition(openPosition), this);
	}

	public Command close() {
		return new InstantCommand(() -> setPosition(closedPosition), this);
	}

	private void setPosition(double position) {
		servoLeft.set(position);
		servoRight.set(position);
	}
}

package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Intakedexer extends SubsystemBase {
	private final MotorEx intake;
	private final MotorEx indexer;
	private final ServoEx brake;
	private final ColorRangeSensor lowerSensor;
	private final ColorRangeSensor upperSensor;

	public Intakedexer(HardwareMap hardwareMap) {
		intake = new MotorEx(hardwareMap, "intake");
		indexer = new MotorEx(hardwareMap, "indexer");
		brake = new ServoEx(hardwareMap, "brake");
		lowerSensor = hardwareMap.get(ColorRangeSensor.class, "lowerSensor");
		upperSensor = hardwareMap.get(ColorRangeSensor.class, "upperSensor");
	}

	public boolean getLowerSensor() {
		return lowerSensor.getDistance(DistanceUnit.CM) < 3;
	}

	public boolean getUpperSensor() {
		return upperSensor.getDistance(DistanceUnit.CM) < 3;
	}

	@Override
	public void periodic() {
	}

}

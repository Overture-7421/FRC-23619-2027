package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class Intakedexer extends SubsystemBase {
	public static double intakePower = 0.7;
	public static double indexerPower = 0.4;
	public static double feedPower = 1.0;
	public static double brakeClosed = 0.5;
	public static double brakeOpen = 1.0;
	public static double lowerSensorCm = 5.5;
	public static double upperSensorCm = 3.5;
	public static double sensorTimeMs = 500.0;

	private final MotorEx intake;
	private final MotorEx indexer;
	private final ServoEx brake;
	private final ColorRangeSensor lowerSensor;
	private final ColorRangeSensor upperSensor;
	private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private boolean intaking = false;
	private boolean shooting = false;
	private boolean outtaking = false;
	private long lowerSince = 0;
	private long upperSince = 0;

	public Intakedexer(HardwareMap hardwareMap) {
		intake = new MotorEx(hardwareMap, "intake");
		indexer = new MotorEx(hardwareMap, "indexer");
		intake.setInverted(true);
		indexer.setInverted(true);
		intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		indexer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		brake = new ServoEx(hardwareMap, "brake");
		lowerSensor = hardwareMap.get(ColorRangeSensor.class, "lowerSensor");
		upperSensor = hardwareMap.get(ColorRangeSensor.class, "upperSensor");
		brake.set(brakeClosed);
	}

	@Override
	public void periodic() {
		if (shooting) {
			brake.set(brakeOpen);
			intake.set(intakePower);
			indexer.set(feedPower);
		} else if (intaking) {
			lowerSince = getLowerSensor() ? (lowerSince == 0 ? System.nanoTime() : lowerSince) : 0;
			upperSince = getUpperSensor() ? (upperSince == 0 ? System.nanoTime() : upperSince) : 0;
			brake.set(brakeClosed);
			indexer.set(isUpperFull() ? 0 : indexerPower);
			intake.set(isFull() ? 0 : intakePower);
		} else if (outtaking) {
			brake.set(brakeClosed);
			indexer.set(-1);
			intake.set(-1);
		} else {
			brake.set(brakeClosed);
			intake.set(0);
			indexer.set(0);
		}

		// telemetry.addData("Intake UpperFull", isUpperFull());
		// telemetry.addData("Intake Full", isFull());
	}

	public Command intake() {
		return new StartEndCommand(() -> intaking = true, () -> intaking = false, this);
	}

	public Command outtake() {
		return new StartEndCommand(() -> outtaking = true, () -> outtaking = false, this);
	}

	public Command shoot() {
		return new StartEndCommand(() -> shooting = true, () -> shooting = false, this);
	}

	public boolean getLowerSensor() {
		return lowerSensor.getDistance(DistanceUnit.CM) < lowerSensorCm;
	}

	public boolean getUpperSensor() {
		return upperSensor.getDistance(DistanceUnit.CM) < upperSensorCm;
	}

	public boolean isUpperFull() {
		return upperSince != 0 && System.nanoTime() - upperSince > sensorTimeMs * 1e6;
	}

	public boolean isFull() {
		return isUpperFull() && lowerSince != 0 && System.nanoTime() - lowerSince > sensorTimeMs * 1e6;
	}
}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Intakedexer extends SubsystemBase {
    private final MotorEx intake;
    private final MotorEx indexer;
    private final ServoEx brake;
    private final ColorRangeSensor colorSensor;;
    public Intakedexer(HardwareMap hardwareMap){
        intake = new MotorEx(hardwareMap, "intake");
        indexer = new MotorEx(hardwareMap, "indexer");
        brake = new ServoEx(hardwareMap, "brake");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
    }
}

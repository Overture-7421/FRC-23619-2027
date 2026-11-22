package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intakedexer extends SubsystemBase {
    private final MotorEx indexerPhase;
    private final MotorEx intakePhase;
    private  int indexerPower = 0;
    private int intakePower = 0;
    private final ColorRangeSensor sensorUp;
    private final ColorRangeSensor sensorDown;
    public Intakedexer(HardwareMap hardwareMap){
       indexerPhase = new MotorEx(hardwareMap, "indexerPhase");
       intakePhase = new MotorEx(hardwareMap, "intakePhase");
        sensorUp = hardwareMap.get(ColorRangeSensor.class, "colorSensorUp");
        sensorDown = hardwareMap.get(ColorRangeSensor.class, "colorSensorDown");
    }

    @Override
    public void periodic(){
        if(!checkDownSensor()){
            intakePhase.set(intakePower);
        }
        else{
            intakePhase.set(0);
        }

        if(!checkUpSensor()){
            indexerPhase.set(indexerPower);
        }
        else{
            indexerPhase.set(0);
        }
    }
    private boolean checkDownSensor(){
        return sensorDown.getDistance(DistanceUnit.CM) < 2.0;
    }
    private boolean checkUpSensor(){
        return sensorUp.getDistance(DistanceUnit.CM) < 2.0;
    }
    public Command intake(){
        return new InstantCommand(() -> {
            indexerPower = 1;
            intakePower = 1;
        });

    }

    public Command reverseIntake(){
        return new InstantCommand(()-> {
            indexerPower = -1;
            intakePower = -1;
        });
    }

    public Command stopIntake(){
        return new InstantCommand(()-> {
            indexerPower = 0;
            intakePower = 0;
        });
    }
}

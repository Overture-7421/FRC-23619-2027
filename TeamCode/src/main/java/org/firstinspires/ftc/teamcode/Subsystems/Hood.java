package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Hood extends SubsystemBase {
    private final ServoEx hood;

    public Hood(HardwareMap hardwareMap){
        hood = new ServoEx(hardwareMap, "hood");
    }

    public Command Close(){
        return new InstantCommand(() -> hood.set(0));
    }

    public Command Mid(){
        return new InstantCommand(() -> hood.set(0.5));
    }

    public Command Far(){
        return new InstantCommand(() -> hood.set(1));
    }
}

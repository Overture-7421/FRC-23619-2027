package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Hood extends SubsystemBase {
    private final ServoEx hood;

    public Hood(HardwareMap hardwareMap){
        hood = new ServoEx(hardwareMap, "hood");
    }
}

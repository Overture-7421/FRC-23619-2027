package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Arm extends SubsystemBase {
    private final ServoEx colibri;

    public Arm(HardwareMap hardwareMap){
        colibri = new ServoEx(hardwareMap, "colibri");
    }
}

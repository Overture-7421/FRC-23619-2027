package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

public class Turret extends SubsystemBase {
    private final CRServoEx torreta;

    public Turret(HardwareMap hardwareMap){
        torreta = new CRServoEx(hardwareMap, "CRServo");
    }
}

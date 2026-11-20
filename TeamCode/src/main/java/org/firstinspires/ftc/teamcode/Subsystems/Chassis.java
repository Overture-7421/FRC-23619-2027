package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class Chassis extends SubsystemBase {
    private final MotorGroup chassisMotors;
    public Chassis(HardwareMap hardwareMap){
        chassisMotors = new MotorGroup(new MotorEx(hardwareMap, "backLeft"),
                new MotorEx(hardwareMap, "frontLeft"),
                new MotorEx(hardwareMap, "frontRight"),
                new MotorEx(hardwareMap, "backRight")
        );
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class Shooter extends SubsystemBase {
    private final MotorGroup shooterMotors;
    public Shooter(HardwareMap hardwareMap){
        shooterMotors = new MotorGroup(new MotorEx(hardwareMap, "leftMotor"),
                new MotorEx(hardwareMap, "rightMotor")
        );
    }
}

package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

public class ConvertPoses {

    public static Pose convertToPedro(Pose pose) {
        return new Pose(
                pose.y() + 72,
                72 - pose.x(),
                pose.heading() - Math.PI / 2
        );
    }
}

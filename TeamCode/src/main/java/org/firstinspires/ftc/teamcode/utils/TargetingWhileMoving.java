package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class TargetingWhileMoving {
    private final InterpLUT distanceToTravelTime;
    private Pose targetLocation = new Pose(0, 0);

    public TargetingWhileMoving(InterpLUT distanceToTravelTime) {
        this.distanceToTravelTime = distanceToTravelTime;
    }

    public void setTargetLocation(Pose targetLocation) {
        this.targetLocation = targetLocation;
    }

    public Pose getMovingTarget(Pose shooterPose, double vx, double vy) {
        double shotTime = distanceToTravelTime.get(distance(targetLocation, shooterPose));

        for (int i = 0; ; i++) {
            Pose testGoal = new Pose(targetLocation.x() - shotTime * vx, targetLocation.y() - shotTime * vy);
            double newShotTime = distanceToTravelTime.get(distance(testGoal, shooterPose));

            if (i == 4 || Math.abs(newShotTime - shotTime) <= 0.010) {
                return testGoal;
            }
            shotTime = newShotTime;
        }
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }
}

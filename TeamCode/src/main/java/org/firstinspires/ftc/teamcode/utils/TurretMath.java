package org.firstinspires.ftc.teamcode.utils;

public final class TurretMath {
    public static final double GEAR_TURRET_TOOTH = 176.0;
    public static final double GEAR_ENCODER1 = 28.0;
    public static final double GEAR_ENCODER2 = 32.0;

    public static final double SLOPE =
            (GEAR_ENCODER2 * GEAR_ENCODER1) / ((GEAR_ENCODER1 - GEAR_ENCODER2) * GEAR_TURRET_TOOTH);
    public static final double GEAR_RATIO_TURRET_TO_ENCODER1 = GEAR_TURRET_TOOTH / GEAR_ENCODER1;
    public static final double DEGREES_PER_ENCODER1_ROTATION = 360.0 / GEAR_RATIO_TURRET_TO_ENCODER1;
    public static final double UNIQUE_RANGE_DEG = 360.0 * Math.abs(SLOPE);

    public static double encoderDegrees(int pulseUs, int minUs, int maxUs, boolean inverted, double offsetTurns) {
        double turns = (pulseUs - minUs) / (double) (maxUs - minUs + 1);
        turns = Math.max(0.0, Math.min(turns, 0.999999));
        turns = (inverted ? -turns : turns) + offsetTurns;
        turns -= Math.floor(turns);
        return turns >= 1.0 ? 0.0 : turns * 360.0;
    }

    public static double calculateTurretAngleDegrees(double encoder1, double encoder2) {
        double difference = encoder2 - encoder1;
        if (difference > 180.0) {
            difference -= 360.0;
        }
        if (difference < -180.0) {
            difference += 360.0;
        }

        double estimatedTurretAngle = difference * SLOPE;
        double encoder1RotationsFloored = Math.floor(estimatedTurretAngle * GEAR_RATIO_TURRET_TO_ENCODER1 / 360.0);
        double turretAngle = (encoder1RotationsFloored * 360.0 + encoder1) / GEAR_RATIO_TURRET_TO_ENCODER1;

        double error = turretAngle - estimatedTurretAngle;
        if (error < -DEGREES_PER_ENCODER1_ROTATION / 2.0) {
            turretAngle += DEGREES_PER_ENCODER1_ROTATION;
        } else if (error > DEGREES_PER_ENCODER1_ROTATION / 2.0) {
            turretAngle -= DEGREES_PER_ENCODER1_ROTATION;
        }
        return turretAngle;
    }

    public static double encoder2ResidualDegrees(double turretAngle, double encoder2) {
        return wrap180(encoder2 - turretAngle * GEAR_TURRET_TOOTH / GEAR_ENCODER2);
    }

    public static double convertToClosestBoundedTurretAngleDegrees(
            double targetAngleDegrees, double currentTotalDegrees, double forwardLimit, double reverseLimit) {
        double closestOffset = targetAngleDegrees - currentTotalDegrees % 360.0;
        if (closestOffset > 180.0) {
            closestOffset -= 360.0;
        } else if (closestOffset < -180.0) {
            closestOffset += 360.0;
        }

        double finalOffset = currentTotalDegrees + closestOffset;
        if ((currentTotalDegrees + closestOffset) % 360.0 == (currentTotalDegrees - closestOffset) % 360.0) {
            if (finalOffset > 0.0) {
                finalOffset = currentTotalDegrees - Math.abs(closestOffset);
            } else {
                finalOffset = currentTotalDegrees + Math.abs(closestOffset);
            }
        }
        if (finalOffset > forwardLimit) {
            finalOffset -= 360.0;
        } else if (finalOffset < reverseLimit) {
            finalOffset += 360.0;
        }
        return finalOffset;
    }

    public static double wrap180(double deg) {
        return deg - 360.0 * Math.floor(deg / 360.0 + 0.5);
    }
}

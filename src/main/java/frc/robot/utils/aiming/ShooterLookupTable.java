package frc.robot.utils.aiming;

import java.util.Arrays;

public class ShooterLookupTable {
    private final double[][] rows;

    public record ShotSetpoint(double shooterRPM, double hoodPercent) {
    }

    public ShooterLookupTable(double[][] sourceRows) {
        if (sourceRows == null || sourceRows.length == 0) {
            throw new IllegalArgumentException("Shooter lookup table must contain at least one row");
        }

        rows = new double[sourceRows.length][3];
        for (int i = 0; i < sourceRows.length; i++) {
            if (sourceRows[i] == null || sourceRows[i].length < 3) {
                throw new IllegalArgumentException("Each shooter lookup row must have 3 columns");
            }
            rows[i][0] = sourceRows[i][0];
            rows[i][1] = sourceRows[i][1];
            rows[i][2] = sourceRows[i][2];
        }

        Arrays.sort(rows, (a, b) -> Double.compare(a[0], b[0]));
    }

    public ShotSetpoint sample(double distanceMeters) {
        if (rows.length == 1) {
            return new ShotSetpoint(rows[0][1], rows[0][2]);
        }

        if (distanceMeters <= rows[0][0]) {
            return new ShotSetpoint(rows[0][1], rows[0][2]);
        }

        int lastIndex = rows.length - 1;
        if (distanceMeters >= rows[lastIndex][0]) {
            return new ShotSetpoint(rows[lastIndex][1], rows[lastIndex][2]);
        }

        for (int i = 0; i < rows.length - 1; i++) {
            double lowerDistance = rows[i][0];
            double upperDistance = rows[i + 1][0];
            if (distanceMeters >= lowerDistance && distanceMeters <= upperDistance) {
                double t = (distanceMeters - lowerDistance) / (upperDistance - lowerDistance);
                return new ShotSetpoint(
                        interpolate(rows[i][1], rows[i + 1][1], t),
                        interpolate(rows[i][2], rows[i + 1][2], t));
            }
        }

        return new ShotSetpoint(rows[lastIndex][1], rows[lastIndex][2]);
    }

    private double interpolate(double a, double b, double t) {
        return a + (b - a) * t;
    }
}

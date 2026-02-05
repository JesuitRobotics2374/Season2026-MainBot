package frc.robot.utils;

import org.apache.commons.math4.legacy.fitting.WeightedObservedPoints;
import org.apache.commons.math4.legacy.fitting.PolynomialCurveFitter;

public class Ballistics {
    // Solving constants
    private static double x; // meters (m) - forward (front +)
    private static double y; // meters (m) - side (left +)
    private static double z; // meters (m) - up (up +)
    private static double theta; // radians (rad)

    private static double Vx; // meters per second (m/s)
    private static double Vy; // meters per second (m/s)
    private static double Vz; // meters per second (m/s)

    private static double Ax; // meters per second squared (m/s^2)
    private static double Ay; // meters per second squared (m/s^2)
    private static double Az; // meters per second squared (m/s^2)

    private static double time; // seconds (s)

    // Other constants
    private static final double deltaH = 1; // meters (m)
    private static final double lambda = 45 * (Math.PI) / 180; // radians (rad)

    private static final double m = 0.215; // kilograms (kg)
    private static final double g = -9.8; // meters per second squared (m/s^2)

    private static final double c = 0.39; // dimensionless (drag coefficient)
    private static final double rho = 1.225; // kilograms per cubic meter (kg/m^3)
    private static final double area = 0.0177;// square meters (m^2)

    private static final double drag = 0.5 * c * rho * area; // drag constant

    private static final double dt = 0.001; // seconds (s)

    public static double calculateX(double Vi, double ViRobotX, double ViRobotY) {
        boolean hasReachedPeak = false;

        x = 0;
        y = 0;
        z = 0;
        theta = 0;

        Vx = Vi * Math.cos(lambda) + ViRobotX;
        Vy = ViRobotY;
        Vz = Vi * Math.sin(lambda);

        double Vel = Math.sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

        Ax = -(drag * Vel * Vx) / m;
        Ay = -(drag * Vel * Vy) / m;
        Az = g - (drag * Vel * Vz) / m;

        time = 0;

        while (!hasReachedPeak) {
            time += dt;

            Vx += Ax * dt;
            Vy += Ay * dt;
            Vz += Az * dt;

            Vel = Math.sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

            Ax = -(drag * Vel * Vx) / m;
            Ay = -(drag * Vel * Vy) / m;
            Az = g - (drag * Vel * Vz) / m;

            x += Vx * dt;
            y += Vy * dt;
            z += Vz * dt;

            hasReachedPeak = Vz <= 0;
        }

        while (z > deltaH) {
            time += dt;

            Vx += Ax * dt;
            Vy += Ay * dt;
            Vz += Az * dt;

            Vel = Math.sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

            Ax = -(drag * Vel * Vx) / m;
            Ay = -(drag * Vel * Vy) / m;
            Az = g - (drag * Vel * Vz) / m;

            x += Vx * dt;
            y += Vy * dt;
            z += Vz * dt;
        }

        return x;
    }

    public static double calculateY(double Vi, double ViRobotX, double ViRobotY) {
        boolean hasReachedPeak = false;

        x = 0;
        y = 0;
        z = 0;
        theta = 0;

        Vx = Vi * Math.cos(lambda) + ViRobotX;
        Vy = ViRobotY;
        Vz = Vi * Math.sin(lambda);

        double Vel = Math.sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

        Ax = -(drag * Vel * Vx) / m;
        Ay = -(drag * Vel * Vy) / m;
        Az = g - (drag * Vel * Vz) / m;

        time = 0;

        while (!hasReachedPeak) {
            time += dt;

            Vx += Ax * dt;
            Vy += Ay * dt;
            Vz += Az * dt;

            Vel = Math.sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

            Ax = -(drag * Vel * Vx) / m;
            Ay = -(drag * Vel * Vy) / m;
            Az = g - (drag * Vel * Vz) / m;

            x += Vx * dt;
            y += Vy * dt;
            z += Vz * dt;

            hasReachedPeak = Vz <= 0;
        }

        while (z > deltaH) {
            time += dt;

            Vx += Ax * dt;
            Vy += Ay * dt;
            Vz += Az * dt;

            Vel = Math.sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

            Ax = -(drag * Vel * Vx) / m;
            Ay = -(drag * Vel * Vy) / m;
            Az = g - (drag * Vel * Vz) / m;

            x += Vx * dt;
            y += Vy * dt;
            z += Vz * dt;
        }

        return y;
    }

    public static double CalculateNeededShooterSpeed(double distanceX, double VRobotX, double VRobotY) {
        double low = 5.0; // minimum shooter speed
        double high = 20.0; // maximum shooter speed
        double tolerance = 0.0001; // acceptable error in meters
        double mid = 0;

        double iterations = 0;

        while (high - low > 1e-5) { // stop when interval is very small
            iterations++;

            mid = (low + high) / 2.0;
            double x = calculateX(mid, VRobotX, VRobotY);

            if (x < distanceX - tolerance) {
                low = mid; // need more speed
            } else if (x > distanceX + tolerance) {
                high = mid; // too fast
            } else {
                break; // within tolerance
            }
        }
        return mid;
    }

    public static void main(String[] args) {

        System.out.println(CalculateNeededShooterSpeed(5, 2, 3));

        // double ViMin = 5; // minimum initial velocity (m/s)
        // double ViMax = 15; // maximum initial velocity (m/s)
        // double step = 0.01; // step size in m/s

        // int numPoints = (int) ((ViMax - ViMin) / step) + 1;
        // double[] velocities = new double[numPoints];
        // double[] deltaX = new double[numPoints];

        // for (int i = 0; i < numPoints; i++) {
        // double Vi = ViMin + i * step;
        // velocities[i] = Vi;
        // deltaX[i] = calculateX(Vi);
        // }

        // // Map Δx -> Vi for polynomial fitting
        // WeightedObservedPoints points = new WeightedObservedPoints();
        // for (int i = 0; i < numPoints; i++) {
        // points.add(deltaX[i], velocities[i]); // Δx as x, Vi as y
        // }

        // // Fit cubic polynomial
        // PolynomialCurveFitter fitter = PolynomialCurveFitter.create(4);
        // double[] coeff = fitter.fit(points.toList());

        // System.out.println("Quartic fit (x -> Vi):");
        // System.out.printf(
        // "Vi ≈ %.6f + %.6f*x + %.6f*x^2 + %.6f*x^3 + %.6f*x^4%n",
        // coeff[0], coeff[1], coeff[2], coeff[3], coeff[4]);

        // double xTarget = 8.0;

        // double ViEstimate = coeff[0]
        // + coeff[1] * xTarget
        // + coeff[2] * xTarget * xTarget
        // + coeff[3] * xTarget * xTarget * xTarget
        // + coeff[4] * xTarget * xTarget * xTarget * xTarget;

        // System.out.println("Estimated Vi for x = " + xTarget + " m: " + ViEstimate +
        // " m/s");

        // System.out.println("x for estimated vi: " + calculateX(ViEstimate));

        // System.out.println(calculateY(ViEstimate, 1));
    }
}

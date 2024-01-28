package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CubicPath {

    public Bezier[] beziers = new Bezier[3];
    public double guessT = 0, arcLength = 0;
    private double totalArcLength;
    double[] arcLengths = new double[beziers.length];
    public CubicPath(Vector2d A1, Vector2d A2, Vector2d A3, Vector2d A4, Vector2d B1, Vector2d B2, Vector2d B3, Vector2d B4, Vector2d C1, Vector2d C2, Vector2d C3, Vector2d C4) {
        beziers[0] = new Bezier(A1, A2, A3, A4);
        beziers[1] = new Bezier(B1, B2, B3, B4);
        beziers[2] = new Bezier(C1, C2, C3, C4);

        calculateTotalArcLength();
    }

    public void setControlPoints(Vector2d A1, Vector2d A2, Vector2d A3, Vector2d A4, Vector2d B1, Vector2d B2, Vector2d B3, Vector2d B4, Vector2d C1, Vector2d C2, Vector2d C3, Vector2d C4) {
        beziers[0].setControlPoints(A1, A2, A3, A4);
        beziers[1].setControlPoints( B1, B2, B3, B4);
        beziers[2].setControlPoints(C1, C2, C3, C4);
        calculateTotalArcLength();
    }

    public Vector2d getPoint(double T) {
        if (T < 0) { T = 0; }
        if (T >= 3) { T = 2.9999; }
        return beziers[(int) T].getPoint(T - Math.floor(T));
    }

    public Vector2d getNormalizedTangent(double T) { return beziers[(int) T].getNormalizedTangent(T - Math.floor(T)); }

    public Vector2d getNormalizedNormal(double T) { return beziers[(int) T].getNormalizedNormal(T - Math.floor(T)); }

    public double getTotalArcLength() {
        return totalArcLength;
    }

    public void calculateTotalArcLength() {
        double total = 0;
        int i = 0;
        for (Bezier bezier : beziers) {
            double length = bezier.getTotalArcLength();
            total += length;
            arcLengths[i] = length;
            i++;
        }
        totalArcLength = total;
    }

    public int whichBezierFromDistance(double distance) {
        if (distance <= arcLengths[0]) {
            return 0;
        }
        else if (/*arcLengths[0] <= distance &&*/ distance <= arcLengths[0] + arcLengths[1]) {
            return 1;
        }
        else if (arcLengths[0] + arcLengths[1] <= distance && distance <= arcLengths[0] + arcLengths[1] + arcLengths[2]) {
            return 2;
        }
        return -1;
    }

    public double distanceToT(double distance) {
        int bezier = whichBezierFromDistance(distance);
        double minus = 0;
        if (bezier == 1) { minus = arcLengths[0]; }
        if (bezier == 2) { minus = arcLengths[0] + arcLengths[1]; }
        return (beziers[bezier].distanceToT(distance - minus)) + bezier;
    }

    public Vector2d findClosestPointOnPath(Vector2d Robot) {
        for (int i = 0; i <= 10; i++) {
            Vector2d guess = getPoint(guessT);
            Vector2d robotVector = new Vector2d(Robot.getX() - guess.getX(), Robot.getY() - guess.getY());
            Vector2d normalizedTangent = getNormalizedTangent(guessT);
            double totalArcLength = getTotalArcLength();
            arcLength += normalizedTangent.dot(robotVector);
            if (arcLength < 0) { guessT = 0; arcLength = 0; }
            if (arcLength >= totalArcLength) { arcLength = totalArcLength - 0.01; }
            guessT = distanceToT(arcLength);
            if (guessT > 2.999) { guessT = 2.999; }
        }
        return getPoint(guessT);
    }

}

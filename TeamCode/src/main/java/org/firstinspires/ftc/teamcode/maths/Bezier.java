package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bezier {

    private Vector2d A,B,C,D;
    public Vector2d[] lookup = new Vector2d[76];
    private double totalArcLength;

    public Bezier(Vector2d A, Vector2d B, Vector2d C, Vector2d D) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;

        generateLookup();
        calculateTotalArcLength();
    }

    public void setControlPoints(Vector2d A, Vector2d B, Vector2d C, Vector2d D) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;

        generateLookup();
        calculateTotalArcLength();
    }

    public void generateLookup() {
        for (int i = 0; i <= 75; i++) {
            lookup[i] = new Vector2d((double) i / 75, getArcLength((double) i / 75));
        }
    }

    public Vector2d getPoint(double T) {
        double weightA = (-Math.pow((T),3) + 3 * Math.pow((T),2) - 3 * T + 1);
        double weightB = (3 * Math.pow((T),3) - 6 * Math.pow((T),2) + 3 * T);
        double weightC = (-3 * Math.pow((T),3) + 3 * Math.pow((T),2));
        double weightD = Math.pow((T),3);

        return A.times(weightA).plus(B.times(weightB)).plus(C.times(weightC)).plus(D.times(weightD));
    }

    public Vector2d firstDerivative(double T) {
        double weightA = (-3 * Math.pow((T),2) + 6 * T - 3);
        double weightB = (9 * Math.pow((T),2) - 12 * T + 3);
        double weightC = (-9 * Math.pow((T),2) + 6 * T);
        double weightD = (3 * Math.pow((T),2));

        return A.times(weightA).plus(B.times(weightB)).plus(C.times(weightC)).plus(D.times(weightD));
    }

    public Vector2d getNormalizedTangent(double T) {
        Vector2d firstDerivative = firstDerivative(T);
        return firstDerivative.div(firstDerivative.distTo(new Vector2d(0,0)));
    }

    public Vector2d getNormalizedNormal(double T) {
        return getNormalizedTangent(T).rotated(90 / (180 / Math.PI));
    }

    public double getTotalArcLength() {
        return totalArcLength;
    }

    public void calculateTotalArcLength() {
        double total = 0;
        for (double i = 0; i <= 75; i++) {
            total += getPoint(i / 75).distTo(getPoint((i + 1) / 75));
        }
        totalArcLength = total;
    }

    public double getArcLength(double T) {
        double total = 0;
        for (double i = 0; i <= 75; i++) {
            if(i / 75 < T) {
                total += getPoint(i / 75).distTo(getPoint((i + 1) / 75));
            }
            else break;
        }
        return total;
    }

    public double distanceToT(double distance) {
        int index = 0;
        for (int i = 0; i < lookup.length; i++) {
            if (i == lookup.length - 1) { return lookup[i].getX(); }
            if (lookup[i].getY() <= distance && lookup[i + 1].getY() >= distance) {
                index = i;
                break;
            }
        }
        return mathsOperations.interpolate(lookup[index], lookup[index + 1], distance).getX();
    }
}

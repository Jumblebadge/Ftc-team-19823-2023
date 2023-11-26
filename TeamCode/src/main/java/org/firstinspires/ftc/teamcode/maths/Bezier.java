package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Bezier {

    public final Vector2d A,B,C,D;
    public double[] lookup;

    public Bezier(Vector2d A, Vector2d B, Vector2d C, Vector2d D) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;

        generateLookup();
    }

    public void generateLookup() {
        for (int i = 0; i <= 15; i++) {
            lookup[i] = getArcLength((double) i /15);
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
        double weightC = (-9 * Math.pow((T),2) + 3 * T);
        double weightD = (3 * Math.pow((T),2));

        return A.times(weightA).plus(B.times(weightB)).plus(C.times(weightC)).plus(D.times(weightD));
    }

    public Vector2d getNormalizedTangent(double T) {
        return getPoint(T).plus(firstDerivative(T).div(firstDerivative(T).distTo(new Vector2d(0,0))));
    }

    public Vector2d getNormalizedNormal(double T) {
        return getNormalizedTangent(T).rotated(AngleUnit.RADIANS.fromDegrees(90));
    }

    public double getTotalArcLength() {
        double total = 0;
        for (double i = 0; i <= 12; i++) {
            total += getPoint(i / 12).distTo(getPoint(i / 12 + 1));
        }
        return total;
    }

    //TODO fix these 2
    public double getArcLength(double T) {
        return 3;
    }

    public double distanceToT(double distance) {
        return 3;

    }
}

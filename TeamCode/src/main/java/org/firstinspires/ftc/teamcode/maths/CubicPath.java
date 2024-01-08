package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CubicPath {

    public Bezier[] beziers = new Bezier[3];
    public double guessT = 0, arcLength = 0;
    Telemetry telemetry;

    public CubicPath(Telemetry telemetry, Vector2d A1, Vector2d A2, Vector2d A3, Vector2d A4, Vector2d B1, Vector2d B2, Vector2d B3, Vector2d B4, Vector2d C1, Vector2d C2, Vector2d C3, Vector2d C4) {
        beziers[0] = new Bezier(A1, A2, A3, A4);
        beziers[1] = new Bezier(B1, B2, B3, B4);
        beziers[2] = new Bezier(C1, C2, C3, C4);
        this.telemetry = telemetry;
    }

    public void setControlPoints(Vector2d A1, Vector2d A2, Vector2d A3, Vector2d A4, Vector2d B1, Vector2d B2, Vector2d B3, Vector2d B4, Vector2d C1, Vector2d C2, Vector2d C3, Vector2d C4) {
        beziers[0] = new Bezier(A1, A2, A3, A4);
        beziers[1] = new Bezier(B1, B2, B3, B4);
        beziers[2] = new Bezier(C1, C2, C3, C4);
    }

    public Vector2d getPoint(double T) {
        if (T < 0) { T = 0; }
        if (T >= 3) { T = 2.9999; }
        return beziers[(int) T].getPoint(T - Math.floor(T));
    }

    public Vector2d getNormalizedTangent(double T) { return beziers[(int) T].getNormalizedTangent(T - Math.floor(T)); }

    public Vector2d getNormalizedNormal(double T) { return beziers[(int) T].getNormalizedNormal(T - Math.floor(T)); }

    public double getTotalArcLength() {
        double total = 0;
        for (Bezier bezier : beziers) {
            total += bezier.getTotalArcLength();
        }
        return total;
    }

    public int whichBezierFromDistance(double distance) {
        double bezier0TotalArcLength = beziers[0].getTotalArcLength();
        double bezier1TotalArcLength = beziers[1].getTotalArcLength();
        double bezier2TotalArcLength = beziers[2].getTotalArcLength();

        if (distance <= bezier0TotalArcLength) {
            return 0;
        }
        else if (/*bezier0TotalArcLength <= distance &&*/ distance <= bezier0TotalArcLength + bezier1TotalArcLength) {
            return 1;
        }
        else if (bezier0TotalArcLength + bezier1TotalArcLength <= distance && distance <= bezier0TotalArcLength + bezier1TotalArcLength + bezier2TotalArcLength) {
            return 2;
        }
        return -1;
    }

    public double distanceToT(double distance) {
        int bezier = whichBezierFromDistance(distance);
        telemetry.addData("which bexeiar: ", bezier);
        return (beziers[bezier].distanceToT(distance)) + bezier;
    }

    public Vector2d findClosestPointOnPath(Vector2d Robot) {
        for (int i = 0; i <= 10; i++) {
            Vector2d guess = getPoint(guessT);
            Vector2d robotVector = new Vector2d(Robot.getX() - guess.getX(), Robot.getY() - guess.getY());
            Vector2d normalizedTangent = getNormalizedTangent(guessT).minus(guess);
            arcLength += normalizedTangent.dot(robotVector);
            if (arcLength < 0) { guessT = 0; arcLength = 0; }
            guessT = distanceToT(arcLength);
            telemetry.addData("guess",getPoint(guessT));
            telemetry.addData("arc",arcLength);
        }
        telemetry.addData("t",guessT);
        return getPoint(guessT);
    }

}

package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CubicPath {

    public Bezier[] beziers = new Bezier[3];
    public double guessT = 0;
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
        if (distance <= beziers[0].getTotalArcLength()) {
            return 0;
        }
        else if (beziers[0].getTotalArcLength() <= distance && distance <= beziers[1].getTotalArcLength()) {
            return 1;
        }
        else if (beziers[1].getTotalArcLength() <= distance && distance <= beziers[2].getTotalArcLength()) {
            return 2;
        }
        return 300;
    }

    public double distanceToT(double distance) {
        return beziers[whichBezierFromDistance(distance)].distanceToT(distance);
    }

    public Vector2d findClosestPointOnPath(Vector2d Robot) {
        guessT = 0;
        double arcLength = 0;
        for (int i = 0; i <= 10; i++) {
            //TODO wtf why is it negative (check t -> s its def broken)
            Vector2d guess = getPoint(guessT);
            Vector2d robotVector = new Vector2d(Robot.getX() - guess.getX(), Robot.getY() - guess.getY());
            arcLength += guess.dot(robotVector);
            guessT = distanceToT(arcLength);
        }
        telemetry.addData("duess",getPoint(guessT));
        return getPoint(guessT);
    }

}

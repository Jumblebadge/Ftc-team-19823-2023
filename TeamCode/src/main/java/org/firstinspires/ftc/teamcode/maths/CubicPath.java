package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CubicPath {

    public Bezier[] beziers;
    public double guessT;

    public CubicPath(Vector2d A1, Vector2d A2, Vector2d A3, Vector2d A4, Vector2d B1, Vector2d B2, Vector2d B3, Vector2d B4, Vector2d C1, Vector2d C2, Vector2d C3, Vector2d C4) { 
        beziers[0] = new Bezier(A1, A2, A3, A4);
        beziers[1] = new Bezier(B1, B2, B3, B4);
        beziers[2] = new Bezier(C1, C2, C3, C4);
    }

    public Vector2d getPoint(double T) {
        return beziers[(int) Math.floor(T)].getPoint(T);
    }

    public Vector2d getNormalizedTangent(double T) { return beziers[(int) Math.floor(T)].getNormalizedTangent(T); }

    public Vector2d getNormalizedNormal(double T) { return beziers[(int) Math.floor(T)].getNormalizedNormal(T); }

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
        double i = 0;
        Vector2d guess = getPoint(guessT);
        Vector2d robotVector = new Vector2d(Robot.getX() - guess.getX(), Robot.getY() - guess.getY());
        while (!mathsOperations.equals(robotVector.dot(getNormalizedTangent(guessT)), 0, 0.1)) {
            guess = getPoint(guessT);
            robotVector = new Vector2d(Robot.getX() - guess.getX(), Robot.getY() - guess.getY());
            arcLength += guess.dot(robotVector);
            guessT = distanceToT(arcLength);
            i++;
        }
        guess = getPoint(guessT);
        return guess;
    }

}

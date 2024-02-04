package org.firstinspires.ftc.teamcode.maths;

import android.graphics.ImageDecoder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.checkerframework.checker.index.qual.GTENegativeOne;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.PathList;

public class GVF {

    CubicPath path;
    Vector2d R, closestPoint, out, tangent;
    FtcDashboard dashboard;
    PIDcontroller headingPID = new PIDcontroller(0.1,0.001,0,0.75,0.1);
    PIDcontroller xPID = new PIDcontroller(0.5,0,0,0.25, 0.1);
    PIDcontroller yPID = new PIDcontroller(0.5,0,0,0.25, 0.1);
    double Kn, Kf, Ks;
    Vector2d temp;
    Telemetry telemetry;


    public GVF(FtcDashboard dashboard, CubicPath path, double Kn, double Kf, double Ks, Telemetry telemetry) {
        this.path = path;
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
        this.dashboard = dashboard;
        this.telemetry = telemetry;
        calculateGVF(PathList.RedLeftPathToSpikePoints[0]);
    }

    public void setPath(CubicPath path) {
        this.path = path;
    }

    public void tune(double Kn, double Kf, double Ks) {
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
    }

    public double calculateError(Vector2d tangent) {
        double magnitudeOfR = R.distTo(new Vector2d(0,0));
        magnitudeOfR = Math.pow(1.3, magnitudeOfR - 10) - 0.073;
        return magnitudeOfR * -(Math.signum(mathsOperations.cross(R,tangent)));
    }

    public void calculateEverything(Vector2d Robot) {
        closestPoint = path.findClosestPointOnPath(Robot);
        R = new Vector2d(Robot.getX() - closestPoint.getX(), Robot.getY() - closestPoint.getY());
    }

    public double tangentHeading() { return AngleUnit.normalizeDegrees(360 - (tangent.angle() * (180 / Math.PI))); }

    public double distanceFromEnd() { return path.getTotalArcLength() - path.arcLength; }

    public boolean isEnding() { return distanceFromEnd() < Kf; }

    public Vector2d calculateGVF(Vector2d Robot) {
        calculateEverything(Robot);
        tangent = path.getNormalizedTangent(path.guessT);
        Vector2d normal = path.getNormalizedNormal(path.guessT);
        double error = calculateError(tangent);
        out = tangent.minus(normal.times(Kn).times(error));
        telemetry.addData("error", error);
        double max = Math.max(Math.abs(out.getX()), Math.abs(out.getY()));
        if (max > 1) {
            out = new Vector2d(out.getX() / max, out.getY() / max);
        }
        out = out.times(Math.min(1,(path.getTotalArcLength() - path.arcLength) / Kf));
        telemetry.addData("errer",(path.getTotalArcLength() - path.arcLength) / Kf);
        temp = Robot;
        out = new Vector2d(out.getY(), out.getX());
        return out.times(Ks);
    }

    public Vector2d calculatePID(Vector2d robot) {
        return new Vector2d(yPID.pidOut(path.getPoint(2.999).getY() - robot.getY()), xPID.pidOut((path.getPoint(2.999).getX()) - robot.getX()));
    }

    public double headingOut(double heading, double targetHeading) {
        if (isEnding()) return headingPID.pidOut(AngleUnit.normalizeDegrees(targetHeading - heading));
        else return headingPID.pidOut(AngleUnit.normalizeDegrees(tangentHeading() - heading));
    }

    public Vector2d output(Vector2d robot) {
        if (isEnding()) telemetry.addData("isending",distanceFromEnd());
        drawPath(dashboard, path, new Pose2d(robot.getX(), robot.getY()));
        telemetry.addData("tep",temp);
        if (isEnding()) return calculatePID(robot);
        else return calculateGVF(robot);
    }

    public void drawPath(FtcDashboard dash, CubicPath path, Pose2d robot) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#51B53F");
        Vector2d[] points = new Vector2d[30];
        double[] x = new double[points.length];
        double[] y = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            points[i] = path.getPoint((double) i * 3 / points.length);
            x[i] = points[i].getX();
            y[i] = points[i].getY();
        }
        canvas.strokePolyline(x,y);
        //drawRobot(canvas, robot, out, path);
        drawPoint(canvas, new Pose2d(closestPoint.getX(), closestPoint.getY()));
        drawVectors(canvas, new Pose2d(closestPoint.getX(), closestPoint.getY()), out, path);
        dash.sendTelemetryPacket(packet);
    }

    public static void drawPoint(Canvas canvas, Pose2d pose) {
        canvas.fillCircle(pose.getX(), pose.getY(), 4);
    }

    public static void drawVectors(Canvas canvas, Pose2d pose, Vector2d out, CubicPath path) {
        Vector2d v = out.times(12);
        double x1 = pose.getX(), y1 = pose.getY();
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
        Vector2d tangent = path.getNormalizedTangent(path.guessT);
        Vector2d normal = path.getNormalizedNormal(path.guessT);
        canvas.setStroke("#B53F51");
        double x11 = path.getPoint(path.guessT).getX(), y11 = path.getPoint(path.guessT).getY();
        Vector2d v1 = tangent.times(6);
        double x12 = path.getPoint(path.guessT).getX() + v1.getX(), y12 = path.getPoint(path.guessT).getY() + v1.getY();
        canvas.strokeLine(x11, y11, x12, y12);
        double x21 = path.getPoint(path.guessT).getX(), y21 = path.getPoint(path.guessT).getY();
        Vector2d v2 = normal.times(6);
        double x22 = path.getPoint(path.guessT).getX() + v2.getX(), y22 = path.getPoint(path.guessT).getY() + v2.getY();
        canvas.strokeLine(x21, y21, x22, y22);
    }

}

package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.checkerframework.checker.index.qual.GTENegativeOne;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GVF {

    CubicPath path;
    Vector2d R, closestPoint, out;
    FtcDashboard dashboard;
    double Kn;
    Telemetry telemetry;

    public GVF(FtcDashboard dashboard, CubicPath path, double Kn, Telemetry telemetry) {
        this.path = path;
        this.Kn = Kn;
        this.dashboard = dashboard;
        this.telemetry = telemetry;
    }

    public void setKn(double Kn) {
        this.Kn = Kn;
    }

    public double calculateError(Vector2d tangent) {
        double magnitudeOfR = R.distTo(new Vector2d(0,0));
        return magnitudeOfR * -(Math.signum(mathsOperations.cross(R,tangent)));
    }

    public void calculateEverything(Vector2d Robot) {
        closestPoint = path.findClosestPointOnPath(Robot);
        R = new Vector2d(Robot.getX() - closestPoint.getX(), Robot.getY() - closestPoint.getY());
    }

    public Vector2d output(Vector2d Robot) {
        calculateEverything(Robot);
        Vector2d tangent = path.getNormalizedTangent(path.guessT);
        Vector2d normal = path.getNormalizedNormal(path.guessT);
        double error = calculateError(tangent);
        out = tangent.minus(normal.times(Kn).times(error));
        telemetry.addData("error", error);
        double max = Math.max(Math.abs(out.getX()), Math.abs(out.getY()));
        drawPath(dashboard, path, new Pose2d(Robot.getX(), Robot.getY()));
        if (max > 1) {
            out = new Vector2d(out.getX() / max, out.getY() / max);
        }
        return out.div(1);
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
        drawRobot(canvas, robot, out, path);
        drawRobot(canvas, new Pose2d(closestPoint.getX(), closestPoint.getY()), out, path);
        dash.sendTelemetryPacket(packet);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, Vector2d out, CubicPath path) {
        drawVectors(canvas, pose, out, path);
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
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

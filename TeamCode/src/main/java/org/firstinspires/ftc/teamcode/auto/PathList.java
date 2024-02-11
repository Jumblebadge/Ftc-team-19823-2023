package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.maths.CubicPath;

public class PathList {

    public static Vector2d Temp = new Vector2d(0,0);

    public static CubicPath justPID(Vector2d vec) {
        Vector2d[] list = new Vector2d[]{vec, vec, vec, vec, vec, vec, vec, vec.plus(new Vector2d(0.05,0.05))};
        return new CubicPath(list);
    }

    public static final Vector2d[] RedRightPathToSpikePoints = {
            new Vector2d(12,-60),
            new Vector2d(12,-57),
            new Vector2d(12,-52),
            new Vector2d(12,-47),
            new Vector2d(12,-39),
            new Vector2d(12,-37),
            new Vector2d(12,-37),
            new Vector2d(12,-34)
    };
    public static final CubicPath RedRightPathToSpike = new CubicPath(RedRightPathToSpikePoints);

    public static final Vector2d[] RedRightSpikeToStackPoints = {
            RedRightPathToSpikePoints[7].plus(new Vector2d(0,-2)),
            new Vector2d(12.5,-47),
            new Vector2d(37.3,-45.8),
            new Vector2d(34.3,-26.6),
            new Vector2d(-8.1,-13.6),
            new Vector2d(-22,-15),
            new Vector2d(-45.8,-16.3),
            new Vector2d(-45,-14)
    };
    public static final CubicPath RedRightSpikeToStack = new CubicPath(RedRightSpikeToStackPoints);

    public static final Vector2d[] RedStackAdjustmentPoints = {
            RedRightSpikeToStackPoints[7],
            new Vector2d(-56.1,-14),
            new Vector2d(-56.2,-14),
            new Vector2d(-56.3,-14),
            new Vector2d(-56.4,-14),
            new Vector2d(-56.5,-14),
            new Vector2d(-56.6,-14),
            new Vector2d(-59,-14)
    };
    public static final CubicPath RedStackAdjustment = new CubicPath(RedStackAdjustmentPoints);

    public static final Vector2d[] RedStackToBoardPoints = {
            RedStackAdjustmentPoints[7],
            new Vector2d(-39.3,-13.4),
            new Vector2d(-37.8,-13.8),
            new Vector2d(-33.0,-14.0),
            new Vector2d(-11.5,-15.6),
            new Vector2d(2.0,-15.4),
            new Vector2d(15.0,-16.0),
            new Vector2d(35.0,-16.0),
    };
    public static final CubicPath RedStackToBoard = new CubicPath(RedStackToBoardPoints);

    public static final Vector2d[] RedBoardAdjustmentPoints = {
            RedStackToBoardPoints[7],
            new Vector2d(35.7,-23.0),
            new Vector2d(34.8,-20.2),
            new Vector2d(35.0,-27.0),
            new Vector2d(34.8,-30.8),
            new Vector2d(35.0,-32.8),
            new Vector2d(34.0,-35.6),
            new Vector2d(54,-38)
    };
    public static final CubicPath RedBoardAdjustment = new CubicPath(RedBoardAdjustmentPoints);

    public static final Vector2d[] RedParkPoints = {
            RedBoardAdjustmentPoints[7],
            new Vector2d(43.7,-36.3),
            new Vector2d(39.0,-36.0),
            new Vector2d(39.7,-27.8),
            new Vector2d(39.0,-22.2),
            new Vector2d(40.5,-18.2),
            new Vector2d(47.5,-8.3),
            new Vector2d(55.0,-12.0),
    };
    public static final CubicPath RedPark = new CubicPath(RedParkPoints);

    public static final Vector2d[] UShapePoints = {
        new Vector2d(-45,-60),
        new Vector2d(20,-60),
        new Vector2d(25,-60),
        new Vector2d(25,-45),
        new Vector2d(25, -30),
        new Vector2d(25,-22),
        new Vector2d(20,-12),
        new Vector2d(-45,-12)
    };
    public static final CubicPath UShapePath = new CubicPath(UShapePoints);

}

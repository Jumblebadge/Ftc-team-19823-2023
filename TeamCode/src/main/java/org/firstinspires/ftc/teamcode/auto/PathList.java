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

    //red right

    public static final Vector2d[] RedRightPathToSpikePoints = {
            new Vector2d(12,-60),
            new Vector2d(12,-57),
            new Vector2d(12,-52),
            new Vector2d(12,-47),
            new Vector2d(12,-39),
            new Vector2d(12,-37),
            new Vector2d(12,-37),
            new Vector2d(12,-32.5)
    };
    public static final CubicPath RedRightPathToSpike = new CubicPath(RedRightPathToSpikePoints);

    public static final Vector2d[] RedRightAlternatePathToSpikePoints = {
            new Vector2d(12,-60),
            new Vector2d(9,-57),
            new Vector2d(9,-52),
            new Vector2d(9,-47),
            new Vector2d(9,-39),
            new Vector2d(9,-37),
            new Vector2d(9,-37),
            new Vector2d(9,-32.5)
    };
    public static final CubicPath RedRightAlternatePathToSpike = new CubicPath(RedRightAlternatePathToSpikePoints);

    public static final Vector2d[] RedRightSpikeToBoardPoints = {
            RedRightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,-60.2),
            new Vector2d(25.4,-43.4),
            new Vector2d(28.8,-42.8),
            new Vector2d(36.0,-40.2),
            new Vector2d(39.6,-39.5),
            new Vector2d(45.7,-37.7),
            new Vector2d(50,-36),
    };
    public static final CubicPath RedRightSpikeToBoard = new CubicPath(RedRightSpikeToBoardPoints);

    public static final Vector2d[] RedRightSpikeToBoardRightPoints = {
            RedRightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,-60.2),
            new Vector2d(25.4,-43.4),
            new Vector2d(28.8,-42.8),
            new Vector2d(36.0,-40.2),
            new Vector2d(39.6,-39.5),
            new Vector2d(45.7,-37.7),
            new Vector2d(50,-41.5),
    };
    public static final CubicPath RedRightSpikeToBoardRight = new CubicPath(RedRightSpikeToBoardRightPoints);

    public static final Vector2d[] RedRightSpikeToBoardLeftPoints = {
            RedRightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,-60.2),
            new Vector2d(25.4,-43.4),
            new Vector2d(28.8,-42.8),
            new Vector2d(36.0,-40.2),
            new Vector2d(39.6,-39.5),
            new Vector2d(45.7,-37.7),
            new Vector2d(50,-28.5),
    };
    public static final CubicPath RedRightSpikeToBoardLeft = new CubicPath(RedRightSpikeToBoardLeftPoints);



    public static final Vector2d[] RedRightSpikeToStackPoints = {
            RedRightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.5,-47),
            new Vector2d(37.3,-45.8),
            new Vector2d(34.3,-26.6),
            new Vector2d(-8.1,-13.6),
            new Vector2d(-22,-15),
            new Vector2d(-45.8,-16.3),
            new Vector2d(-45,-14)
    };
    public static final CubicPath RedRightSpikeToStack = new CubicPath(RedRightSpikeToStackPoints);

    //red left

    public static final Vector2d[] RedLeftPathToSpikePoints = {
            new Vector2d(-36,-60),
            new Vector2d(-36,-57),
            new Vector2d(-36,-52),
            new Vector2d(-36,-47),
            new Vector2d(-36,-39),
            new Vector2d(-36,-37),
            new Vector2d(-36,-37),
            new Vector2d(-36,-32.5)
    };
    public static final CubicPath RedLeftPathToSpike = new CubicPath(RedLeftPathToSpikePoints);

    public static final Vector2d[] RedLeftAlternatePathToSpikePoints = {
            new Vector2d(-36,-60),
            new Vector2d(-39,-57),
            new Vector2d(-39,-52),
            new Vector2d(-39,-47),
            new Vector2d(-39,-39),
            new Vector2d(-39,-37),
            new Vector2d(-39,-37),
            new Vector2d(-39,-32.5)
    };
    public static final CubicPath RedLeftAlternatePathToSpike = new CubicPath(RedLeftAlternatePathToSpikePoints);

    public static final Vector2d[] RedLeftSpikeToBoardPoints = {
            RedLeftPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(-35.8,-54.2),
            new Vector2d(-57.0,-47.0),
            new Vector2d(-57.5,-27.2),
            new Vector2d(-1.0,-15.7),
            new Vector2d(8.7,-15.7),
            new Vector2d(22.0,-15.2),
            new Vector2d(35.0,-16),
    };
    public static final CubicPath RedLeftSpikeToBoard = new CubicPath(RedLeftSpikeToBoardPoints);

    //red general

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
            new Vector2d(35.2,-18.2),
            new Vector2d(35.0,-21.0),
            new Vector2d(36.0,-23.2),
            new Vector2d(37.4,-27.5),
            new Vector2d(40.4,-30.8),
            new Vector2d(45.7,-33.8),
            new Vector2d(50,-36)
    };
    public static final CubicPath RedBoardAdjustment = new CubicPath(RedBoardAdjustmentPoints);

    public static final Vector2d[] RedBoardAdjustmentRightPoints = {
            RedStackToBoardPoints[7],
            new Vector2d(35.2,-18.2),
            new Vector2d(35.0,-21.0),
            new Vector2d(36.0,-23.2),
            new Vector2d(37.4,-27.5),
            new Vector2d(40.4,-30.8),
            new Vector2d(45.7,-33.8),
            new Vector2d(50,-47)
    };
    public static final CubicPath RedBoardAdjustmentRight = new CubicPath(RedBoardAdjustmentRightPoints);

    public static final Vector2d[] RedBoardAdjustmentLeftPoints = {
            RedStackToBoardPoints[7],
            new Vector2d(35.2,-18.2),
            new Vector2d(35.0,-21.0),
            new Vector2d(36.0,-23.2),
            new Vector2d(37.4,-27.5),
            new Vector2d(40.4,-30.8),
            new Vector2d(45.7,-33.8),
            new Vector2d(50,-26.5)
    };
    public static final CubicPath RedBoardAdjustmentLeft = new CubicPath(RedBoardAdjustmentLeftPoints);

    public static final Vector2d[] RedParkPoints = {
            RedBoardAdjustmentPoints[7],
            new Vector2d(43.7,-36.3),
            new Vector2d(39.0,-36.0),
            new Vector2d(39.7,-27.8),
            new Vector2d(39.0,-22.2),
            new Vector2d(40.5,-18.2),
            new Vector2d(47.5,-8.3),
            new Vector2d(48,-13.0),
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

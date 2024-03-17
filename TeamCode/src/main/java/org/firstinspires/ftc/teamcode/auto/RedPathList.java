package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.maths.CubicPath;

public class RedPathList {

    public static Vector2d Temp = new Vector2d(0,0);

    public static CubicPath justPID(Vector2d vec) {
        Vector2d[] list = new Vector2d[]{vec, vec, vec, vec, vec, vec, vec, vec.plus(new Vector2d(0.05,0.05))};
        return new CubicPath(list);
    }

    //red right

    public static final Vector2d[] RightPathToSpikePoints = {
            new Vector2d(12,-60),
            new Vector2d(12,-57),
            new Vector2d(12,-52),
            new Vector2d(12,-47),
            new Vector2d(12,-39),
            new Vector2d(12,-37),
            new Vector2d(12,-37),
            new Vector2d(12,-32.5)
    };
    public static final CubicPath RightPathToSpike = new CubicPath(RightPathToSpikePoints);

    public static final Vector2d[] RightPathToLeftSpikePoints = {
            new Vector2d(12,-60),
            new Vector2d(9,-57),
            new Vector2d(9,-52),
            new Vector2d(9,-47),
            new Vector2d(9,-39),
            new Vector2d(9,-37),
            new Vector2d(8,-37),
            new Vector2d(8,-32.5)
    };
    public static final CubicPath RightPathToLeftSpike = new CubicPath(RightPathToLeftSpikePoints);
    public static final Vector2d[] RightPathToRightSpikePoints = {
            new Vector2d(12,-60),
            new Vector2d(15,-57),
            new Vector2d(15,-52),
            new Vector2d(15,-47),
            new Vector2d(15,-39),
            new Vector2d(15,-37),
            new Vector2d(15.75,-37),
            new Vector2d(15.75 ,-32.5)
    };
    public static final CubicPath RightPathToRightSpike = new CubicPath(RightPathToRightSpikePoints);

    public static final Vector2d[] RightSpikeToBoardPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.0,-41.4),
            new Vector2d(13.3,-50.0),
            new Vector2d(17.3,-52.0),
            new Vector2d(26.2,-54.0),
            new Vector2d(30.3,-52.2),
            new Vector2d(38.0,-43.8),
            new Vector2d(36,-36),
    };
    public static final CubicPath RightSpikeToBoard = new CubicPath(RightSpikeToBoardPoints);

    public static final Vector2d[] RightSpikeToBoardRightPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,-60.2),
            new Vector2d(25.4,-43.4),
            new Vector2d(28.8,-42.8),
            new Vector2d(36.0,-40.2),
            new Vector2d(39.6,-39.5),
            new Vector2d(45.7,-37.7),
            new Vector2d(50,-41.5),
    };
    public static final CubicPath RightSpikeToBoardRight = new CubicPath(RightSpikeToBoardRightPoints);

    public static final Vector2d[] RightSpikeToBoardLeftPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,-60.2),
            new Vector2d(25.4,-43.4),
            new Vector2d(28.8,-42.8),
            new Vector2d(36.0,-40.2),
            new Vector2d(39.6,-39.5),
            new Vector2d(45.7,-37.7),
            new Vector2d(50,-28.5),
    };
    public static final CubicPath RightSpikeToBoardLeft = new CubicPath(RightSpikeToBoardLeftPoints);


    public static final Vector2d[] RightSpikeToStackPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.5,-47),
            new Vector2d(37.3,-45.8),
            new Vector2d(34.3,-26.6),
            new Vector2d(-8.1,-13.6),
            new Vector2d(-22,-15),
            new Vector2d(-45.8,-16.3),
            new Vector2d(-45,-14)
    };
    public static final CubicPath RightSpikeToStack = new CubicPath(RightSpikeToStackPoints);

    //red left

    public static final Vector2d[] LeftPathToSpikePoints = {
            new Vector2d(-36,-60),
            new Vector2d(-36,-57),
            new Vector2d(-36,-52),
            new Vector2d(-36,-47),
            new Vector2d(-36,-39),
            new Vector2d(-36,-37),
            new Vector2d(-36,-37),
            new Vector2d(-36,-33.5)
    };
    public static final CubicPath LeftPathToSpike = new CubicPath(LeftPathToSpikePoints);

    public static final Vector2d[] LeftReversePathToSpikePoints = {
            new Vector2d(-36,-33.5),
            new Vector2d(-36,-33.6),
            new Vector2d(-36,-33.7),
            new Vector2d(-36,-33.8),
            new Vector2d(-36,-34),
            new Vector2d(-36,-35),
            new Vector2d(-36,-36),
            new Vector2d(-36,-37)
    };
    public static final CubicPath LeftReversePathToSpike = new CubicPath(LeftReversePathToSpikePoints);

    public static final Vector2d[] LeftPathToLeftSpikePoints = {
            new Vector2d(-36,-60),
            new Vector2d(-39,-57),
            new Vector2d(-39,-52),
            new Vector2d(-39,-47),
            new Vector2d(-39,-39),
            new Vector2d(-39,-37),
            new Vector2d(-40,-37),
            new Vector2d(-40,-32.5)
    };
    public static final CubicPath LeftPathToLeftSpike = new CubicPath(LeftPathToLeftSpikePoints);

    public static final Vector2d[] LeftPathToRightSpikePoints = {
            new Vector2d(-36,-60),
            new Vector2d(-33,-57),
            new Vector2d(-33,-52),
            new Vector2d(-33,-47),
            new Vector2d(-33,-39),
            new Vector2d(-33,-37),
            new Vector2d(-32,-37),
            new Vector2d(-32,-32.5)
    };
    public static final CubicPath LeftPathToRightSpike = new CubicPath(LeftPathToRightSpikePoints);

    public static final Vector2d[] LeftSpikeToBoardPoints = {
            LeftPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(-42.0,-43.8),
            new Vector2d(-52.5,-47.0),
            new Vector2d(-58.0,-32.0),
            new Vector2d(-32.4,-11.2),
            new Vector2d(-5.3,-11.0),
            new Vector2d(40.4,-12.0),
            new Vector2d(36.0,-36.0),
    };
    public static final CubicPath LeftSpikeToBoard = new CubicPath(LeftSpikeToBoardPoints);

    //red general

    public static final Vector2d[] StackAdjustmentPoints = {
            RightSpikeToStackPoints[7],
            new Vector2d(-56.1,-14),
            new Vector2d(-56.2,-14),
            new Vector2d(-56.3,-14),
            new Vector2d(-56.4,-14),
            new Vector2d(-56.5,-14),
            new Vector2d(-56.6,-14),
            new Vector2d(-59,-14)
    };
    public static final CubicPath StackAdjustment = new CubicPath(StackAdjustmentPoints);

    public static final Vector2d[] StackToBoardPoints = {
            StackAdjustmentPoints[7],
            new Vector2d(-39.3,-13.4),
            new Vector2d(-37.8,-13.8),
            new Vector2d(-33.0,-14.0),
            new Vector2d(-11.5,-15.6),
            new Vector2d(2.0,-15.4),
            new Vector2d(15.0,-16.0),
            new Vector2d(35.0,-16.0),
    };
    public static final CubicPath StackToBoard = new CubicPath(StackToBoardPoints);

    public static final Vector2d[] BoardAdjustmentPoints = {
            RightSpikeToBoardPoints[7],
            new Vector2d(40.7,-36.2),
            new Vector2d(37.0,-36.0),
            new Vector2d(39.3,-36.0),
            new Vector2d(41.6,-36.0),
            new Vector2d(45.0,-36.0),
            new Vector2d(47.4,-35.8),
            new Vector2d(49.5,-36.0),
    };
    public static final CubicPath BoardAdjustment = new CubicPath(BoardAdjustmentPoints);

    public static final Vector2d[] BoardAdjustmentRightPoints = {
            RightSpikeToBoardPoints[7],
            new Vector2d(40.7,-36.2),
            new Vector2d(37.0,-36.0),
            new Vector2d(39.3,-36.0),
            new Vector2d(41.6,-36.0),
            new Vector2d(44.7,-36.6),
            new Vector2d(49.6,-40.0),
            new Vector2d(49.5,-46.0),
    };
    public static final CubicPath BoardAdjustmentRight = new CubicPath(BoardAdjustmentRightPoints);

    public static final Vector2d[] BoardAdjustmentLeftPoints = {
            RightSpikeToBoardPoints[7],
            new Vector2d(40.7,-36.2),
            new Vector2d(37.0,-36.0),
            new Vector2d(39.3,-36.0),
            new Vector2d(41.6,-36.0),
            new Vector2d(44.7,-35.5),
            new Vector2d(50.0,-31.7),
            new Vector2d(49.5,-26.5),
    };
    public static final CubicPath BoardAdjustmentLeft = new CubicPath(BoardAdjustmentLeftPoints);

    public static final Vector2d[] ParkMidPoints = {
            BoardAdjustmentRightPoints[7],
            new Vector2d(48.4,-42.9),
            new Vector2d(48.8,-37.95),
            new Vector2d(48.65,-32.4),
            new Vector2d(48.4,-24.15),
            new Vector2d(48.9,-16.6),
            new Vector2d(58.2,-11.2),
            new Vector2d(60.0,-12.0),
    };
    public static final CubicPath ParkMid = new CubicPath(ParkMidPoints);

    public static final Vector2d[] ParkSidePoints = {
            new Vector2d(51.5, -19),
            new Vector2d(51.5,-27.7),
            new Vector2d(50.5,-34.4),
            new Vector2d(49.2,-39.3),
            new Vector2d(49.0,-46.7),
            new Vector2d(49.6,-50.2),
            new Vector2d(53.3,-56.0),
            new Vector2d(60.0,-60.0),
    };
    public static final CubicPath ParkSide = new CubicPath(ParkSidePoints);

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

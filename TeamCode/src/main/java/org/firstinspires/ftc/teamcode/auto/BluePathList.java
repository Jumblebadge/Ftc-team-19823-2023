package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.maths.CubicPath;

public class BluePathList {

    public static Vector2d Temp = new Vector2d(0,0);

    public static CubicPath justPID(Vector2d vec) {
        Vector2d[] list = new Vector2d[]{vec, vec, vec, vec, vec, vec, vec, vec.plus(new Vector2d(0.05,0.05))};
        return new CubicPath(list);
    }

    //red right

    public static final Vector2d[] RightPathToSpikePoints = {
            new Vector2d(12,60),
            new Vector2d(12,57),
            new Vector2d(12,52),
            new Vector2d(12,47),
            new Vector2d(12,39),
            new Vector2d(12,37),
            new Vector2d(12,37),
            new Vector2d(12,32.5)
    };
    public static final CubicPath RightPathToSpike = new CubicPath(RightPathToSpikePoints);

    public static final Vector2d[] RightPathToLeftSpikePoints = {
            new Vector2d(12,60),
            new Vector2d(9,57),
            new Vector2d(9,52),
            new Vector2d(9,47),
            new Vector2d(9,39),
            new Vector2d(9,37),
            new Vector2d(8,37),
            new Vector2d(8,32.5)
    };
    public static final CubicPath RightPathToLeftSpike = new CubicPath(RightPathToLeftSpikePoints);
    public static final Vector2d[] RightPathToRightSpikePoints = {
            new Vector2d(12,60),
            new Vector2d(15,57),
            new Vector2d(15,52),
            new Vector2d(15,47),
            new Vector2d(15,39),
            new Vector2d(15,37),
            new Vector2d(16,37),
            new Vector2d(16,32.5)
    };
    public static final CubicPath RightPathToRightSpike = new CubicPath(RightPathToRightSpikePoints);

    public static final Vector2d[] RightSpikeToBoardPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,60.2),
            new Vector2d(25.4,43.4),
            new Vector2d(28.8,42.8),
            new Vector2d(36.0,40.2),
            new Vector2d(39.6,39.5),
            new Vector2d(45.7,37.7),
            new Vector2d(50.0,36),
    };
    public static final CubicPath RightSpikeToBoard = new CubicPath(RightSpikeToBoardPoints);

    public static final Vector2d[] RightSpikeToBoardRightPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,60.2),
            new Vector2d(25.4,43.4),
            new Vector2d(28.8,42.8),
            new Vector2d(36.0,40.2),
            new Vector2d(39.6,39.5),
            new Vector2d(45.7,37.7),
            new Vector2d(50.0,41.5),
    };
    public static final CubicPath RightSpikeToBoardRight = new CubicPath(RightSpikeToBoardRightPoints);

    public static final Vector2d[] RightSpikeToBoardLeftPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.6,60.2),
            new Vector2d(25.4,43.4),
            new Vector2d(28.8,42.8),
            new Vector2d(36.0,40.2),
            new Vector2d(39.6,39.5),
            new Vector2d(45.7,37.7),
            new Vector2d(50.0,28.5),
    };
    public static final CubicPath RightSpikeToBoardLeft = new CubicPath(RightSpikeToBoardLeftPoints);



    public static final Vector2d[] RightSpikeToStackPoints = {
            RightPathToSpikePoints[7].plus(new Vector2d(0,10)),
            new Vector2d(12.5,47),
            new Vector2d(37.3,45.8),
            new Vector2d(34.3,26.6),
            new Vector2d(-8.1,13.6),
            new Vector2d(-22,15),
            new Vector2d(-45.8,16.3),
            new Vector2d(-45,14)
    };
    public static final CubicPath RightSpikeToStack = new CubicPath(RightSpikeToStackPoints);

    //red left

    public static final Vector2d[] LeftPathToSpikePoints = {
            new Vector2d(-36,60),
            new Vector2d(-36,57),
            new Vector2d(-36,52),
            new Vector2d(-36,47),
            new Vector2d(-36,39),
            new Vector2d(-36,37),
            new Vector2d(-36,37),
            new Vector2d(-36,33.5)
    };
    public static final CubicPath LeftPathToSpike = new CubicPath(LeftPathToSpikePoints);

    public static final Vector2d[] LeftPathToLeftSpikePoints = {
            new Vector2d(-36,60),
            new Vector2d(-39,57),
            new Vector2d(-39,52),
            new Vector2d(-39,47),
            new Vector2d(-39,39),
            new Vector2d(-39,37),
            new Vector2d(-40,37),
            new Vector2d(-40,32.5)
    };
    public static final CubicPath LeftPathToLeftSpike = new CubicPath(LeftPathToLeftSpikePoints);

    public static final Vector2d[] LeftPathToRightSpikePoints = {
            new Vector2d(-36,60),
            new Vector2d(-33,57),
            new Vector2d(-33,52),
            new Vector2d(-33,47),
            new Vector2d(-33,39),
            new Vector2d(-33,37),
            new Vector2d(-32,37),
            new Vector2d(-32,32.5)
    };
    public static final CubicPath LeftPathToRightSpike = new CubicPath(LeftPathToRightSpikePoints);

    public static final Vector2d[] LeftSpikeToBoardPoints = {
            LeftPathToSpikePoints[7].plus(new Vector2d(0,20)),
            new Vector2d(-35.8,54.2),
            new Vector2d(-57.0,47.0),
            new Vector2d(-57.5,27.2),
            new Vector2d(-1.0,15.7),
            new Vector2d(8.7,15.7),
            new Vector2d(22.0,15.2),
            new Vector2d(35.0,16),
    };
    public static final CubicPath LeftSpikeToBoard = new CubicPath(LeftSpikeToBoardPoints);

    //red general

    public static final Vector2d[] StackAdjustmentPoints = {
            RightSpikeToStackPoints[7],
            new Vector2d(-56.1,14),
            new Vector2d(-56.2,14),
            new Vector2d(-56.3,14),
            new Vector2d(-56.4,14),
            new Vector2d(-56.5,14),
            new Vector2d(-56.6,14),
            new Vector2d(-59.0,14)
    };
    public static final CubicPath StackAdjustment = new CubicPath(StackAdjustmentPoints);

    public static final Vector2d[] StackToBoardPoints = {
            StackAdjustmentPoints[7],
            new Vector2d(-39.3,13.4),
            new Vector2d(-37.8,13.8),
            new Vector2d(-33.0,14.0),
            new Vector2d(-11.5,15.6),
            new Vector2d( 2.0,15.4),
            new Vector2d(15.0,16.0),
            new Vector2d(35.0,16.0),
    };
    public static final CubicPath StackToBoard = new CubicPath(StackToBoardPoints);

    public static final Vector2d[] BoardAdjustmentPoints = {
            StackToBoardPoints[7],
            new Vector2d(35.2,18.2),
            new Vector2d(35.0,21.0),
            new Vector2d(36.0,23.2),
            new Vector2d(37.4,27.5),
            new Vector2d(40.4,30.8),
            new Vector2d(45.7,33.8),
            new Vector2d(50.0,35)
    };
    public static final CubicPath BoardAdjustment = new CubicPath(BoardAdjustmentPoints);

    public static final Vector2d[] BoardAdjustmentRightPoints = {
            StackToBoardPoints[7],
            new Vector2d(35.2,18.2),
            new Vector2d(35.0,21.0),
            new Vector2d(36.0,23.2),
            new Vector2d(37.4,27.5),
            new Vector2d(40.4,30.8),
            new Vector2d(45.7,33.8),
            new Vector2d(50.0,46)
    };
    public static final CubicPath BoardAdjustmentRight = new CubicPath(BoardAdjustmentRightPoints);

    public static final Vector2d[] BoardAdjustmentLeftPoints = {
            StackToBoardPoints[7],
            new Vector2d(35.2,18.2),
            new Vector2d(35.0,21.0),
            new Vector2d(36.0,23.2),
            new Vector2d(37.4,27.5),
            new Vector2d(40.4,30.8),
            new Vector2d(45.7,33.8),
            new Vector2d(50.0,26.5)
    };
    public static final CubicPath BoardAdjustmentLeft = new CubicPath(BoardAdjustmentLeftPoints);

    public static final Vector2d[] ParkPoints = {
            BoardAdjustmentPoints[7],
            new Vector2d(43.7,36.3),
            new Vector2d(39.0,36.0),
            new Vector2d(39.7,27.8),
            new Vector2d(39.0,22.2),
            new Vector2d(40.5,18.2),
            new Vector2d(47.5,8.3),
            new Vector2d(48.0,13.0),
    };
    public static final CubicPath Park = new CubicPath(ParkPoints);


}
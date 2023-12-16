package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class DetectElement implements VisionProcessor {

    //TODO wtf is wrong

    Rect left = new Rect(50, 90, 50, 30);
    Rect middle = new Rect(150, 90, 50, 30);
    Rect right = new Rect(230, 100, 50, 30);

    private State detected = State.MID;

    Telemetry telemetry;

    public enum State {
        RIGHT,
        MID,
        LEFT
    }

    Mat submat = new Mat();

    public DetectElement(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        double leftSaturation = getSaturation(frame, left);
        telemetry.addData("leftsat",leftSaturation);
        double midSaturation = getSaturation(frame, middle);
        double rightSaturation = getSaturation(frame, right);

        if (leftSaturation > midSaturation && leftSaturation > rightSaturation) {
            return State.LEFT;
        } else if (rightSaturation > leftSaturation && rightSaturation > midSaturation) {
            return State.RIGHT;
        }
        return State.MID;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.BLACK);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint detectedPaint = new Paint(paint);
        detectedPaint.setColor(Color.GREEN);

        detected = (State) userContext;
        telemetry.addData("et",userContext);
        telemetry.update();
        switch(detected) {
            case LEFT:
                canvas.drawRect(makeRect(left,scaleBmpPxToCanvasPx),detectedPaint);
                canvas.drawRect(makeRect(middle,scaleBmpPxToCanvasPx),paint);
                canvas.drawRect(makeRect(right,scaleBmpPxToCanvasPx),paint);
                break;
            case MID:
                canvas.drawRect(makeRect(left,scaleBmpPxToCanvasPx),paint);
                canvas.drawRect(makeRect(middle,scaleBmpPxToCanvasPx),detectedPaint);
                canvas.drawRect(makeRect(right,scaleBmpPxToCanvasPx),paint);
                break;
            case RIGHT:
                canvas.drawRect(makeRect(left,scaleBmpPxToCanvasPx),paint);
                canvas.drawRect(makeRect(middle,scaleBmpPxToCanvasPx),paint);
                canvas.drawRect(makeRect(right,scaleBmpPxToCanvasPx),detectedPaint);
                break;
        }
    }

    private double getSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeRect(Rect rect, float scale) {
        int left = Math.round(rect.x * scale);
        int top = Math.round(rect.y * scale);
        int right = left + Math.round(rect.width * scale);
        int bottom = top + Math.round(rect.height * scale);

        return new android.graphics.Rect(left,top,right,bottom);
    }

    public State returnDetected() {
        return detected;
    }

}
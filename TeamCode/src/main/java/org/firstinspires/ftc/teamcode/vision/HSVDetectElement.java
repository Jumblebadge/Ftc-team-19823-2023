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
import org.opencv.imgproc.Imgproc;

public class HSVDetectElement implements VisionProcessor {

    Rect left = new Rect(130, 270, 70, 100);
    Rect middle = new Rect(370, 250, 100, 80);
    Rect right = new Rect(620, 270, 70, 100);

    private static State detected = State.MID;

    public enum State {
        RIGHT,
        MID,
        LEFT
    }

    Mat submat = new Mat();
    Mat HSVmat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, HSVmat, Imgproc.COLOR_RGB2HSV);

        double leftSaturation = getSaturation(HSVmat, left);
        double midSaturation = getSaturation(HSVmat, middle);
        double rightSaturation = getSaturation(HSVmat, right);

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

        Paint detectedPaint = new Paint();
        detectedPaint.setColor(Color.GREEN);
        detectedPaint.setStyle(Paint.Style.STROKE);
        detectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        detected = (State) userContext;
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
        Scalar saturation = Core.mean(submat);
        return saturation.val[1];
    }

    private android.graphics.Rect makeRect(Rect rect, float scale) {
        int left = Math.round(rect.x * scale);
        int top = Math.round(rect.y * scale);
        int right = left + Math.round(rect.width * scale);
        int bottom = top + Math.round(rect.height * scale);

        return new android.graphics.Rect(left,top,right,bottom);
    }

    public static State returnDetected() {
        return detected;
    }

}
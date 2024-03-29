package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//OpenCV code that may or may not work
//Test github commit
public class VisionPipeline_RED_SCUFF extends OpenCvPipeline
{
    Telemetry tel;
    public VisionPipeline_RED_SCUFF(Telemetry tel)
    {
        this.tel=tel;
    }
    public enum SkystonePosition
    {
        LEFT,
        CENTER,
        RIGHT
    }


    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,120);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(120,120);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(260,120);
    static final int REGION_WIDTH = 60;
    static final int REGION_HEIGHT = 90;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Red, region2_Red, region3_Red;
    Mat region1_Blue, region2_Blue, region3_Blue;
    Mat region1_Green, region2_Green, region3_Green;
    Mat RGB = new Mat();
    Mat Red = new Mat();
    Mat Blue =new Mat();
    Mat Green =new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile SkystonePosition position = SkystonePosition.LEFT;
    private volatile int pos;
    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, RGB, Imgproc.COLOR_RGB2BGR);
        Core.extractChannel(RGB, Red, 0);
        Core.extractChannel(RGB, Green, 1);
        Core.extractChannel(RGB, Blue, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Red = Red.submat(new Rect(region1_pointA, region1_pointB));
        region2_Red = Red.submat(new Rect(region2_pointA, region2_pointB));
        region3_Red = Red.submat(new Rect(region3_pointA, region3_pointB));

        region1_Blue = Blue.submat(new Rect(region1_pointA, region1_pointB));
        region2_Blue = Blue.submat(new Rect(region2_pointA, region2_pointB));
        region3_Blue = Blue.submat(new Rect(region3_pointA, region3_pointB));

        region1_Green = Green.submat(new Rect(region1_pointA, region1_pointB));
        region2_Green = Green.submat(new Rect(region2_pointA, region2_pointB));
        region3_Green = Green.submat(new Rect(region3_pointA, region3_pointB));


    }
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including SkyStones
         * (because SkyStones have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the SkyStone to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the SkyStone.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        inputToCb(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */

        int counter1 = 0, counter2 = 0, counter3 = 0;

        //int redValue = 0, greenValue = 0, blueValue = 0
        for(int i = 0; i < REGION_HEIGHT; i++) {
            for (int j = 0; j < REGION_WIDTH; j++) {
                counter1 += region1_Red.get(i, j)[0];
                counter2 += region2_Red.get(i, j)[0];
                counter3 += region3_Red.get(i, j)[0];
                //if(region1_Red.get(i, j)[0] < 125 && region1_Green.get(i, j)[1] > 95 && region1_Blue.get(i, j)[2] > 70) counter1++;
                //if(region2_Red.get(i, j)[0] > 150 && region2_Green.get(i, j)[1] < 100 && region2_Blue.get(i, j)[2] > 135) counter2++;
                //if(region3_Red.get(i, j)[0] > 150 && region3_Green.get(i, j)[1] < 100 && region3_Blue.get(i, j)[2] > 135) counter3++;
                //if (region1_Blue.get(i, j)[0] > region1_Green.get(i, j)[0] + region1_Red.get(i, j)[0] && region1_Blue.get(i, j)[0] > 100) counter1++;
                //if (region2_Blue.get(i, j)[0] > region2_Green.get(i, j)[0] + region2_Red.get(i, j)[0] && region2_Blue.get(i, j)[0] > 100) counter2++;
                //if (region3_Blue.get(i, j)[0] > region3_Green.get(i, j)[0] + region3_Red.get(i, j)[0] && region3_Blue.get(i, j)[0] > 100) counter3++;

                //if(region1_Red.get(i, j, region1_Red))
            }
        }

        //avg1 = (int) (0.8*Core.mean(region1_Cb).val[0])+(int) Core.mean(region1_Cr).val[0];
        //avg2 = (int) (0.8*Core.mean(region2_Cb).val[0])+(int) Core.mean(region2_Cr).val[0];
        //avg3 = (int) (0.8*Core.mean(region3_Cb).val[0])+(int) Core.mean(region3_Cr).val[0];

        tel.addData("Area 1", counter1);
        tel.addData("Area 2", counter2);
        tel.addData("Area 3", counter3);
        //tel.addData("Test", (int) Core.mean(region1_Cb).val[0]);
        tel.addData("Spot", getAnalysis());
        tel.update();
        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */

        int min = Math.min(counter1, counter2);
        int trueMin=Math.min(min, counter3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(trueMin==counter1) // Was it from region 1?
        {
            position = SkystonePosition.LEFT; // Record our analysis
            pos=3;
            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(trueMin==counter2) // Was it from region 2?
        {
            position = SkystonePosition.CENTER; // Record our analysis
            pos=2;
            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        else if(trueMin == counter3) // Was it from region 3?
        {
            position = SkystonePosition.RIGHT; // Record our analysis
            pos=1;
            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public int getAnalysis()
    {
        return pos;
    }
}


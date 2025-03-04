package code.vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import android.util.Size;

public class GameObjectCVProcessor {
    OpenCvCamera camera;
    public BlockCVInterpreterPipeline pipeline;
    private boolean streaming;
    private String device_name;
    protected final Size cameraResolution = new Size(640, 480);;
    protected final OpenCvCameraRotation CAMERA_ROTATION = OpenCvCameraRotation.SENSOR_NATIVE;
    protected final OpenCvWebcam.StreamFormat webcamStreamFormat = OpenCvWebcam.StreamFormat.YUY2;

    public GameObjectCVProcessor(HardwareMap hardwareMap, String device_name) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.device_name = device_name;
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, device_name), cameraMonitorViewId);
        // OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //  OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        this.pipeline = new BlockCVInterpreterPipeline();
        this.setPipeline(this.pipeline);
        this.streaming = false;
    }

    public GameObjectCVProcessor() {
        this.camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        // OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMoni
        this.pipeline = new BlockCVInterpreterPipeline();
        this.setPipeline(this.pipeline);
        this.streaming = false;
    }

    public void setPipeline(BlockCVInterpreterPipeline pipeline) {
        this.pipeline = pipeline;
        this.camera.setPipeline(this.pipeline);
    }


    public String getDeviceName() {
        return this.device_name;
    }

    public boolean isStreaming() {
        return this.streaming;
    }

    public int safeStart() {
        this.streaming = true;
        int err = this.startStream();
        if (err != 0) {
            this.streaming = false;
            return err;
        }
        return 0;
    }

    public int startStream() {
        final int[] exit_code = {0};
        this.streaming = true;
        FtcDashboard.getInstance().startCameraStream(this.camera, 0);
//        this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                /*
//                 * Tell the camera to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Also, we specify the rotation that the camera is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//                exit_code[0] = errorCode;
//            }
//        });
        try
        {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    try
                    {
//                        cameraState = VisionPortal.CameraState.CAMERA_DEVICE_READY;
                        camera.setPipeline(pipeline);

                        if (camera instanceof OpenCvWebcam)
                        {
                            CameraCalibrationIdentity identity = ((OpenCvWebcam) camera).getCalibrationIdentity();

                            if (identity != null)
                            {
//                                calibration = CameraCalibrationHelper.getInstance().getCalibration(identity, cameraResolution.getWidth(), cameraResolution.getHeight());
                            }
                        }

                        if (true)
                        {
//                            cameraState = VisionPortal.CameraState.STARTING_STREAM;

                            if (camera instanceof OpenCvWebcam)
                            {
                                ((OpenCvWebcam)camera).startStreaming(cameraResolution.getWidth(), cameraResolution.getHeight(), CAMERA_ROTATION, webcamStreamFormat); //  == null ? null : webcamStreamFormat.eocvStreamFormat);
                            }
                            else
                            {
                                camera.startStreaming(cameraResolution.getWidth(), cameraResolution.getHeight(), CAMERA_ROTATION);
                            }

//                            cameraState = VisionPortal.CameraState.STREAMING;
                        }
                    }
                    finally
                    {
//                        userStateSemaphore.release();
                    }
                }

                @Override
                public void onError(int errorCode)
                {
                    RobotLog.ee("VisionPortalImpl", "Camera opening failed.");
//                    userStateSemaphore.release();
                }
            });
        }
        catch (Exception e)
        {
//            userStateSemaphore.release();
            throw e;
        }
        return exit_code[0];
    }

    public void stop() {
        this.camera.stopStreaming();
        this.streaming = false;
//        camera.closeCameraDevice();
    }
}
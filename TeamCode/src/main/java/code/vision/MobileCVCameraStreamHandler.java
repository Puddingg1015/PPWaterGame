package code.vision;

public class MobileCVCameraStreamHandler {
    public static void main(String[] args) {
        GameObjectCVProcessor cvProcessor = new GameObjectCVProcessor();
        cvProcessor.startStream();
        System.out.println("Started stream asynchronous thread");
    }

    private GameObjectCVProcessor cvProcessor;

    public MobileCVCameraStreamHandler(GameObjectCVProcessor cvProcessor) {
        this.__forceSetCVStreamProcessor(cvProcessor);
    }

    public boolean setCVStreamProcessor(GameObjectCVProcessor cvProcessor) {
        if (this.cvProcessor.isStreaming()) {
            return false;
        }
        this.__forceSetCVStreamProcessor(cvProcessor);
        return true;
    }

    private void __forceSetCVStreamProcessor(GameObjectCVProcessor cvProcessor) {
        this.cvProcessor = cvProcessor;
    }

    public MobileCVCameraStreamHandler() {
        this(new GameObjectCVProcessor());
    }

    GameObjectCVProcessor getCVProcessor() {
        return this.cvProcessor;
    }

    void toggleCVStream() {
        if (this.cvProcessor.isStreaming()) {
            this.cvProcessor.stop();
        } else {
            this.cvProcessor.startStream();
        }
    }


}

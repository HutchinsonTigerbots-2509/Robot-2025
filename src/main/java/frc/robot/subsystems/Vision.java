// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.common.networktables.PacketSubscriber;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.timesync.TimeSyncSingleton;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.*;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase {
  /** Creates a new Vision. */
  public vision() {

    //     *****     CAMERA SERVER INITIALIZATION     *****     //
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    public class PhotonCamera implements AutoCloseable {
      private static int InstanceCount = 0;
      public static final String kTableName = "photonvision";

      private final NetworkTable cameraTable;
      PacketSubscriber<PhotonPipelineResult> resultSubscriber;
      BooleanPublisher driverModePublisher;
      BooleanSubscriber driverModeSubscriber;
      StringSubscriber versionEntry;
      IntegerEntry inputSaveImgEntry, outputSaveImgEntry;
      IntegerPublisher pipelineIndexRequest, ledModeRequest;
      IntegerSubscriber pipelineIndexState, ledModeState;
      IntegerSubscriber heartbeatEntry; // assuming camera server connection
      DoubleArraySubscriber cameraIntrinsicsSubscriber;
      DoubleArraySubscriber cameraDistortionSubscriber;
      MultiSubscriber topicNameSubscriber;
      NetworkTable rootPhotonTable;

      @Override
      public void close() {
        resultSubscriber.close();
        driverModePublisher.close();
        versionEntry.close();
        inputSaveImgEntry.close();
        outputSaveImgEntry.close();
        pipelineIndexRequest.close();
        pipelineIndexState.close();
        ledModeRequest.close();
        ledModeState.close();
        pipelineIndexRequest.close(); // Closed twice on the official docs. Copying
                                      // for reliability sake.
        cameraIntrinsicsSubscriber.close();
        cameraDistortionSubscriber.close();
        topicNameSubscriber.close();
      }

    //     *****     CAMERA OBJECTS     *****     //
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

      private final String path;
      private final String name;

      private static boolean VERSION_CHECK_ENABLED = true;
      private static long VERSION_CHECK_INTERVAL = 5; // TODO: consider double
      double lastVersionCheckTime = 0; // Delay between checking 'latest version'. Not critical.

      private long prevHeartbeatValue = -1; // if server != connection?
      private double prevHeartbeatChangeTime = 0; // Delay between changing camera serve connections
      private static final double HEARTBEAT_DEBOUNCE_SEC = 0.5; // checks for a server connection every .5 seconds

      double prevTimeSyncWarnTime = 0; // Delay between posting camera server warnings to the console
      private static final double WARN_DEBOUNCE_SEC = 5; // checks for warnings every 5 seconds

      public static void setVersionCheckEnabled(boolean enabled) {
        VERSION_CHECK_ENABLED = enabled;
      }

          /**
     * Constructs a PhotonCamera from a root table.
     *
     * @param instance The NetworkTableInstance to pull data from. This can be a custom instance in
     *     simulation, but should *usually* be the default NTInstance from
     *     NetworkTableInstance::getDefault
     * @param cameraName The name of the camera, as seen in the UI. // TODO:
     */

    public PhotonCamera(NetworkTableInstance instance, String cameraName) {
      name = cameraName; //TODO: fetch camera name
      rootPhotonTable = instance.getTable(kTableName); //TODO: fetch table name 
      this.cameraTable = rootPhotonTable.getSubTable(cameraName);
      path = cameraTable.getPath();
      var rawBytesEntry =
              cameraTable
                      .getRawTopic("rawBytes")
                      .subscribe(
                        PhotonPipelineResult.photonStruct.getTypeString(),
                        new byte[] {},
                        PubSubOption.periodic(0.01), // Periodic call time..?
                        PubSubOption.sendAll(true), // fetch all data from the network table
                        PubSubOption.pollStorage(20));
      resultSubscriber = new PacketSubscriber<>(rawBytesEntry, PhotonPipelineResult.photonStruct);
      driverModePublisher = cameraTable.getBooleanTopic("driverModeRequest").publish();
      driverModeSubscriber = cameraTable.getBooleanTopic("driverMode").subscribe(false);
      inputSaveImgEntry = cameraTable.getIntegerTopic("InputSaveImgCmd").getEntry(0);
      outputSaveImgEntry = cameraTable.getIntegerTopic("outputSaveImgCmd").getEntry(0);
      pipelineIndexRequest = cameraTable.getIntegerTopic("pipelineIndexRequest").publish();
      pipelineIndexState = cameraTable.getIntegerTopic("heartbeat").subscribe(-1);
      heartbeatEntry = cameraTable.getIntegerTopic("heartbeat").subscribe(-1);
      cameraIntrinsicsSubscriber = cameraTable.getDoubleArrayTopic("cameraIntrinsics").subscribe(null);
      cameraDistortionSubscriber = cameraTable.getDoubleArrayTopic("cameraDistortion").subscribe(null);

      ledModeRequest = rootPhotonTable.getIntegerTopic("ledModeRequest").publish();
      ledModeState = rootPhotonTable.getIntegerTopic("ledModeState").subscribe(-1);
      versionEntry = rootPhotonTable.getStringTopic("version").subscribe("");

      topicNameSubscriber = new MultiSubscriber(instance, new String[] {"/photonvision/"}, PubSubOption.topicsOnly(true));
      HAL.report(tResourceType.kResourceType_PhotonCamera, InstanceCount);
      InstanceCount++;

      TimeSyncSingleton.load();
    }

     /**
     * Constructs a PhotonCamera from the name of the camera.
     *
     * @param cameraName The nickname of the camera (found in the PhotonVision UI).
     */
    public PhotonCamera(String cameraName) {
      this(NetworkTableInstance.getDefault(), cameraName);
    }

        /**
     * The list of pipeline results sent by PhotonVision since the last call to getAllUnreadResults().
     * Calling this function clears the internal FIFO queue, and multiple calls to
     * getAllUnreadResults() will return different (potentially empty) result arrays. Be careful to
     * call this exactly ONCE per loop of your robot code! FIFO depth is limited to 20 changes, so
     * make sure to call this frequently enough to avoid old results being discarded, too!
     */
    public List<PhotonPipelineResult> getAllUnreadResults() {
      verifyVersion(); // TODO: initialized later in the program on OFFICIAL docs...

      List<PhotonPipelineResult> ret = new ArrayList<>();

      var changes = resultSubscriber.getAllChanges();
      for (var c : changes) {
        var result = c.value;
        checkTimeSyncOrWarn(result); // TODO: also initialized later on...
        ret.add(result);
      }

      return ret;
    }

    /**
     * Returns the latest pipeline result. This is simply the most recent result Received via NT.
     * Calling this multiple times will always return the most recent result.
     *
     * <p>Replaced by {@link #getAllUnreadResults()} over getLatestResult, as this function can miss
     * results, or provide duplicate ones!
     */
    @Deprecated(since = "2024", forRemoval = true)
    public PhotonPipelineResult getLatestResult() {
      verifyVersion(); // TODO:

      var ret = resultSubscriber.get();

      if (ret.timestamp == 0) return new PhotonPipelineResult();

      var result = ret.value;
      checkTimeSyncOrWarn(result); // TODO: 4 lines.
      return result;
    }

    private void checkTimeSyncOrWarn(PhotonPipelineResult result) {
      if (result.metadata.timeSinceLastPong > 5L * 1000000L) {
        if (Timer.getFPGATimestamp() > (prevTimeSyncWarnTime + WARN_DEBOUNCE_SEC)) {
          prevTimeSyncWarnTime = Timer.getFPGATimestamp();

          DriverStation.reportWarning(
            "PhotonVision coprocessor at path " + path + " is not connected to the TimeSyncServer? It's been "
            + String.format("%.2f", result.metadata.timeSinceLastPong / 1e6)
            + "s since the coprocessor last heard a pong.\n\n"
            + "Check /photonvision/.timesync/{COPROCESSOR_HOSTNAME} for more information",
            false);
        }
      } else {
        prevTimeSyncWarnTime = 0;
      }
    }
     /**
     * Returns whether the camera is in driver mode.
     *
     * @return Whether the camera is in driver mode.
     */
    public boolean getDriverMode() {
      return driverModeSubscriber.get();
    }

        /**
     * Toggles driver mode.
     *
     * @param driverMode Whether to set driver mode.
     */
    public void setDriverMode(boolean driverMode) {
      driverModePublisher.set(driverMode);
    }

    /**
     * Request the camera to save a new image file from the input camera stream with overlays. Images
     * take up space in the filesystem of the PhotonCamera. Calling it frequently will fill up disk
     * space and eventually cause the system to stop working. Clear out images in
     * /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues.
     */
    public void takeInputSnapshot() {
        inputSaveImgEntry.set(inputSaveImgEntry.get() + 1);
    }

    /**
     * Request the camera to save a new image file from the output stream with overlays. Images take
     * up space in the filesystem of the PhotonCamera. Calling it frequently will fill up disk space
     * and eventually cause the system to stop working. Clear out images in
     * /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues.
     */
    public void takeOutputSnapshot() {
        outputSaveImgEntry.set(outputSaveImgEntry.get() + 1);
    }

    /**
     * Returns the active pipeline index.
     *
     * @return The active pipeline index.
     */
    public int getPipelineIndex() {
        return (int) pipelineIndexState.get(0);
    }

    /**
     * Allows the user to select the active pipeline index.
     *
     * @param index The active pipeline index.
     */
    public void setPipelineIndex(int index) {
        pipelineIndexRequest.set(index);
    }

    /**
     * Returns the current LED mode.
     *
     * @return The current LED mode.
     */
    public VisionLEDMode getLEDMode() {
        int value = (int) ledModeState.get(-1);
        return switch (value) {
            case 0 -> VisionLEDMode.kOff;
            case 1 -> VisionLEDMode.kOn;
            case 2 -> VisionLEDMode.kBlink;
            default -> VisionLEDMode.kDefault;
        };
    }

    /**
     * Sets the LED mode.
     *
     * @param led The mode to set to.
     */
    public void setLED(VisionLEDMode led) {
        ledModeRequest.set(led.value);
    }

    /**
     * Returns the name of the camera. This will return the same value that was given to the
     * constructor as cameraName.
     *
     * @return The name of the camera.
     */
    public String getName() {
        return name;
    }

    /**
     * Returns whether the camera is connected and actively returning new data. Connection status is
     * debounced.
     *
     * @return True if the camera is actively sending frame data, false otherwise.
     */
    public boolean isConnected() {
      var curHeartbeat = heartbeatEntry.get();
      var now = Timer.getFPGATimestamp();

      if (curHeartbeat != prevHeartbeatValue) {
        prevHeartbeatChangeTime = now;
        prevHeartbeatValue = curHeartbeat;
      }

      return (now - prevHeartbeatChangeTime) < HEARTBEAT_DEBOUNCE_SEC;
    }

    public Optional<Matrix<N3, N3>> getCameraMatrix() {
      var cameraMatrix = cameraIntrinsicsSubscriber.get();
      if (cameraMatrix != null && cameraMatrix.length == 9) {
        return Optional.of(MatBuilder.fill(Nat.N3(), Nat.N3(), cameraMatrix));
      } else return Optional.empty();
    }

    public Optional<Matrix<N8, N1>> getDistCoeffs() {
      var distCoeffs = cameraDistortionSubscriber.get();
      if (distCoeffs != null && distCoeffs.length <= 8) {
        double[] data = new double[8];
        Arrays.fill(data, 0);
        System.arraycopy(distCoeffs, 0, data, 0, distCoeffs.length);

        return Optional.of(MatBuilder.fill(Nat.N8(), Nat.N1(), data));
      } else return Optional.empty();
    }
    /**
     * Gets the NetworkTable representing this camera's subtable. You probably don't ever need to call
     * this.
     */
    public final NetworkTable getCameraTable() {
      return cameraTable;
    }

    // TODO: finish/verify/init
    // void verifyVersion() {
    //   if (!VERSION_CHECK_ENABLED) return;

    //   if ((Timer.getFPGATimestamp() - lastVersionCheckTime) < VERSION_CHECK_INTERVAL) return;

    //   if (!heartbeatEntry.exists()) {
    //     var cameraNames = getTablesThatLookLikePhotonCameras(); // TODO: hopefully initialized later as well. not checking.
        
    //     if (cameraNames.isEmpty()) {
    //       DriverStation.reportError("Could not find **any** PhotonVision coprocessors on NetworkTables. Double check that Photonvision");
    //     }
    //   }
    // }

   }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

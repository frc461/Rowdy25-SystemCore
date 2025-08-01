package frc.robot.subsystems.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.constants.Constants;

public class WristTelemetry {
    private final Wrist wrist;

    public WristTelemetry(Wrist wrist) {
        this.wrist = wrist;
    }

    private final NetworkTable wristTelemetryTable = Constants.NT_INSTANCE.getTable("WristTelemetry");
    private final DoublePublisher wristPositionPub = wristTelemetryTable.getDoubleTopic("Wrist Position").publish();
    private final DoublePublisher wristTargetPub = wristTelemetryTable.getDoubleTopic("Wrist Target").publish();
    private final DoublePublisher wristErrorPub = wristTelemetryTable.getDoubleTopic("Wrist Error").publish();
    private final StringPublisher wristStatePub = wristTelemetryTable.getStringTopic("Wrist State").publish();
    private final BooleanPublisher wristAtTargetPub = wristTelemetryTable.getBooleanTopic("Wrist At Target").publish();
    private final BooleanPublisher wristNearTargetPub = wristTelemetryTable.getBooleanTopic("Wrist Near Target").publish();
    private final DoublePublisher wristCurrentPub = wristTelemetryTable.getDoubleTopic("Wrist Current").publish();

    public void publishValues() {
        wristPositionPub.set(wrist.getPosition());
        wristTargetPub.set(wrist.getTarget());
        wristErrorPub.set(wrist.getError());
        wristStatePub.set(wrist.getState().toString());
        wristAtTargetPub.set(wrist.isAtTarget());
        wristNearTargetPub.set(wrist.nearTarget());
        wristCurrentPub.set(wrist.getCurrent());

        logValues();
    }

    private void logValues() {
        DogLog.log("WristPosition", wrist.getPosition());
        DogLog.log("WristTarget", wrist.getTarget());
        DogLog.log("WristError", wrist.getError());
        DogLog.log("WristState", wrist.getState().toString());
        DogLog.log("WristIsAtTarget", wrist.isAtTarget());
        DogLog.log("WristNearTarget", wrist.nearTarget());
    }
}

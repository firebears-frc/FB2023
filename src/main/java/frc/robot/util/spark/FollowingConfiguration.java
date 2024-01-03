package frc.robot.util.spark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ExternalFollower;

public interface FollowingConfiguration {
    public void apply(CANSparkMax motor);

    public static FollowingConfiguration sparkMax(int deviceID, boolean inverted) {
        return external(ExternalFollower.kFollowerSpark, deviceID, inverted);
    }

    public static FollowingConfiguration external(ExternalFollower leader, int deviceID, boolean inverted) {
        return new FollowingConfiguration() {
            @Override
            public void apply(CANSparkMax motor) {
                Util.configureCheckAndVerify(ignored -> motor.follow(leader, deviceID, inverted), motor::isFollower,
                        true, "follow");
            }
        };
    }
}

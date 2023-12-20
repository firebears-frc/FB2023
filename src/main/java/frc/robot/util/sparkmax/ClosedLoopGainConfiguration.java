package frc.robot.util.sparkmax;

import com.revrobotics.SparkMaxPIDController;

public class ClosedLoopGainConfiguration {
    private final double p, i, d, ff;
    private final double minOutput, maxOutput;
    private final Double dFilter, iZone, maxIAccum;

    public ClosedLoopGainConfiguration(double p, double i, double d, double ff) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ff = ff;
        this.minOutput = -1.0;
        this.maxOutput = 1.0;
        this.dFilter = null;
        this.iZone = null;
        this.maxIAccum = null;
    }

    public ClosedLoopGainConfiguration(double p, double i, double d, double ff, double minOutput, double maxOutput) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ff = ff;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.dFilter = null;
        this.iZone = null;
        this.maxIAccum = null;
    }

    public ClosedLoopGainConfiguration(double p, double i, double d, double ff, double minOutput, double maxOutput,
            Double dFilter, Double iZone, Double maxIAccum) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ff = ff;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.dFilter = dFilter;
        this.iZone = iZone;
        this.maxIAccum = maxIAccum;
    }

    void apply(SparkMaxPIDController pid) {
        Util.configureCheckAndVerify(pid::setP, pid::getP, p, "p");
        Util.configureCheckAndVerify(pid::setI, pid::getI, i, "i");
        Util.configureCheckAndVerify(pid::setD, pid::getD, d, "d");
        Util.configureCheckAndVerify(pid::setFF, pid::getFF, ff, "ff");
        Util.configureCheckAndVerify(ignored -> pid.setOutputRange(minOutput, maxOutput),
                () -> pid.getOutputMin() == minOutput && pid.getOutputMax() == maxOutput, true, "outputRange");
        if (dFilter != null)
            Util.configureCheckAndVerify(pid::setDFilter, () -> pid.getDFilter(0), dFilter, "dFilter");
        if (iZone != null)
            Util.configureCheckAndVerify(pid::setIZone, pid::getIZone, iZone, "iZone");
        if (maxIAccum != null)
            Util.configureCheckAndVerify(setting -> pid.setIMaxAccum(setting, 0), () -> pid.getIMaxAccum(0), maxIAccum,
                    "maxIAccum");
    }
}

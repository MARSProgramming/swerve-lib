package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;
    private String canivoreName = "";

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public CanCoderFactoryBuilder withCanivoreName(String canivoreName) {
        this.canivoreName = canivoreName;
        return this;
    }

    public boolean useCanivore() {
        // null or empty canivore name means don't use canivore
        return !(canivoreName == null || canivoreName.isEmpty());
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            config.MagnetSensor.MagnetOffset = configuration.getOffset()/Math.PI/2.0;
            config.MagnetSensor.SensorDirection = direction == Direction.CLOCKWISE ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;

            CANcoder encoder = new CANcoder(configuration.getId(), canivoreName);

            CtreUtils.checkCtreError(encoder.getAbsolutePosition().setUpdateFrequency(1.0/(periodMilliseconds/1000.0)), "Failed to configure CANCoder update rate");
            CtreUtils.checkCtreError(encoder.getConfigurator().apply(config, 250), "Failed to configure CANCoder");

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANcoder encoder;

        private EncoderImplementation(CANcoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition().getValueAsDouble() * 360.0);
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}

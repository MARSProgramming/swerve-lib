package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    double getPosition();

    void set(double driveVoltage, double steerAngle);
}

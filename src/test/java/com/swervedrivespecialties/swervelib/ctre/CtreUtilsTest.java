package com.swervedrivespecialties.swervelib.ctre;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.StatusCode;

class CtreUtilsTest {
    @Test
    void checkNeoError() {
        assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(StatusCode.GeneralError, ""));
        assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(StatusCode.FirmVersionCouldNotBeRetrieved, ""));
        assertDoesNotThrow(() -> CtreUtils.checkCtreError(StatusCode.OK, ""));
    }
}

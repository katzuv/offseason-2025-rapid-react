package frc.robot.test

import frc.robot.lib.extensions.*

// Test file to verify KSP processor catches violations
object TestRedundantDecimals {
    // These should trigger compilation errors:
    val violation1 = 2.0.volts
    val violation2 = 0.0.rot
    val violation3 = 3.0.m
    
    // These should pass:
    val valid1 = 2.volts
    val valid2 = 2.5.m
    val valid3 = 1.deg
}

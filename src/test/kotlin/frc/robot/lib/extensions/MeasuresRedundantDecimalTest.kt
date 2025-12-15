package frc.robot.lib.extensions

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.assertContains
import kotlin.test.assertTrue

/**
 * Test that redundant decimal points in integer literals are rejected.
 * 
 * Valid: 1.m, 2.volts, 3.deg
 * Invalid: 1.0.m, 2.0.volts, 3.0.deg (should throw exception)
 * Valid: 1.5.m, 2.7.volts (legitimate decimals, should not throw)
 */
class MeasuresRedundantDecimalTest {
    
    @Test
    fun `Integer literals should work without decimal point`() {
        // These should all work fine
        val d1 = 1.m
        val d2 = 5.cm
        val a1 = 90.deg
        val a2 = 2.rot
        val v = 12.volts
        val i = 20.amps
        val t = 3.sec
        
        // Just verify they don't throw
        assertTrue(d1.toString().contains("Meter"))
        assertTrue(a1.toString().contains("Degree"))
    }
    
    @Test
    fun `Double literals with decimal values should work`() {
        // These should all work fine (legitimate decimals)
        val d1 = 1.5.m
        val d2 = 2.7.cm
        val a1 = 90.5.deg
        val a2 = 3.14159.rad
        val v = 12.6.volts
        
        // Just verify they don't throw
        assertTrue(d1.toString().contains("Meter"))
        assertTrue(a1.toString().contains("Degree"))
    }
    
    @Test
    fun `Double literals with zero decimal should throw exception - Distance`() {
        val exception = assertThrows<IllegalArgumentException> {
            @Suppress("UNUSED_VARIABLE")
            val d = 2.0.m
        }
        assertContains(exception.message!!, "Redundant decimal point")
        assertContains(exception.message!!, "2")
    }
    
    @Test
    fun `Double literals with zero decimal should throw exception - Angle`() {
        val exception = assertThrows<IllegalArgumentException> {
            @Suppress("UNUSED_VARIABLE")
            val a = 0.0.rot
        }
        assertContains(exception.message!!, "Redundant decimal point")
        assertContains(exception.message!!, "0")
    }
    
    @Test
    fun `Double literals with zero decimal should throw exception - Voltage`() {
        val exception = assertThrows<IllegalArgumentException> {
            @Suppress("UNUSED_VARIABLE")
            val v = 3.0.volts
        }
        assertContains(exception.message!!, "Redundant decimal point")
        assertContains(exception.message!!, "3")
    }
    
    @Test
    fun `Double literals with zero decimal should throw exception - Time`() {
        val exception = assertThrows<IllegalArgumentException> {
            @Suppress("UNUSED_VARIABLE")
            val t = 100.0.sec
        }
        assertContains(exception.message!!, "Redundant decimal point")
        assertContains(exception.message!!, "100")
    }
    
    @Test
    fun `Edge case - zero should work as integer`() {
        val d = 0.m
        // Should not throw
        assertTrue(d.toString().contains("Meter"))
    }
}


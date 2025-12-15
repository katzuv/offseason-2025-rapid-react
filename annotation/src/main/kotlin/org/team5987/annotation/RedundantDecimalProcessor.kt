package org.team5987.annotation

import com.google.devtools.ksp.processing.KSPLogger
import com.google.devtools.ksp.processing.Resolver
import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSFile
import com.google.devtools.ksp.symbol.KSNode
import com.google.devtools.ksp.symbol.KSVisitorVoid
import com.google.devtools.ksp.symbol.Location
import java.io.File

/**
 * KSP processor that detects redundant decimal points in integer literals
 * used with unit extension functions (e.g., 2.0.m instead of 2.m)
 */
class RedundantDecimalProcessor(
    private val logger: KSPLogger
) : SymbolProcessor {
    
    private val unitExtensions = setOf(
        "m", "cm", "mm", "meters", "centimeters", "millimeters",
        "deg", "degrees", "rad", "radians", "rot", "rotations",
        "sec", "seconds", "percent", "amps", "volts",
        "kg2m", "kilogramSquareMeters",
        "rps", "mps", "deg_ps", "degreesPerSecond",
        "rad_ps", "radiansPerSecond", "rot_ps", "rotationsPerSecond",
        "mps_ps", "deg_ps_ps", "rps_squared"
    )
    
    // Pattern to match: number.0+.unit (e.g., 2.0.volts, 100.00.m)
    private val pattern = Regex("""(\d+)\.0+\.(${unitExtensions.joinToString("|")})\b""")
    
    override fun process(resolver: Resolver): List<KSAnnotated> {
        // Get all source files in the current compilation
        resolver.getAllFiles().forEach { file ->
            checkFile(file)
        }
        return emptyList()
    }
    
    private fun checkFile(file: KSFile) {
        // Read the actual source file content
        val sourceFile = File(file.filePath)
        if (!sourceFile.exists()) return
        
        val content = sourceFile.readText()
        val lines = content.lines()
        
        // Check each line for violations
        lines.forEachIndexed { lineIndex, line ->
            val matches = pattern.findAll(line)
            matches.forEach { match ->
                val number = match.groupValues[1]
                val unit = match.groupValues[2]
                val lineNumber = lineIndex + 1
                
                // Log error with file location
                logger.error(
                    "Redundant decimal point in integer literal: '${match.value}'. " +
                    "Use '$number.$unit' instead of '${match.value}'.",
                    file
                )
            }
        }
    }
}

/**
 * Provider for RedundantDecimalProcessor
 */
class RedundantDecimalProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return RedundantDecimalProcessor(environment.logger)
    }
}

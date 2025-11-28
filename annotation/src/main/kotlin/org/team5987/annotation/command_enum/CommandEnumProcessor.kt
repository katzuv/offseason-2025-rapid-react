package org.team5987.annotation.command_enum

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*

const val ANNOTATION_PACKAGE = "org.team5987.annotation.create_command.CreateCommand"

class CreateCommandProcessor(
    private val env: SymbolProcessorEnvironment
) : SymbolProcessor {
    private val code = env.codeGenerator

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols = resolver.getSymbolsWithAnnotation(ANNOTATION_PACKAGE)

        symbols
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_CLASS }
            .forEach { enumDecl ->
                generateForEnum(enumDecl)
            }

        return emptyList()
    }

    private fun generateForEnum(enumDecl: KSClassDeclaration) {
        val pkg = enumDecl.packageName.asString()
        val enumName = enumDecl.simpleName.asString()

        val entries = enumDecl.declarations
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_ENTRY }
            .map { it.simpleName.asString() }

        val fileName = "${enumName}CommandFactory"
        val file = code.createNewFile(
            Dependencies(false),
            pkg,
            fileName
        )

        val classContent = buildString {
            appendLine("package $pkg")
            appendLine()
            appendLine()
            appendLine("import edu.wpi.first.wpilibj2.command.Command")
            appendLine()
            appendLine("interface $fileName {")
            appendLine()

            for (entry in entries) {
                appendLine("    fun ${entry.lowercase()}(): Command =")
                appendLine("        setAngle($enumName.$entry)")
                appendLine()
            }

            appendLine("    fun setAngle(value: $enumName): Command")
            appendLine("}")
        }

        file.write(classContent.toByteArray())
    }
}

class CommandEnumProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return CreateCommandProcessor(environment)
    }
}
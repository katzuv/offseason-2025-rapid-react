package org.team5987.annotation.create_command

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*

const val ANNOTATION_PACKAGE = "org.team5987.annotation.create_command.CreateCommand"

class CreateCommandProcessor(
    private val env: SymbolProcessorEnvironment
) : SymbolProcessor {

    private val logger = env.logger
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
            .filter { it is KSClassDeclaration || it.classKind == ClassKind.ENUM_ENTRY || it.simpleName.asString().isNotBlank() }
            .map { it.simpleName.asString() }
            .filter { it.isNotEmpty() }

        val fileName = "${enumName}Provider"
        val file = code.createNewFile(
            Dependencies(false),
            pkg,
            fileName
        )

        val classContent = buildString {
            appendLine("package $pkg")
            appendLine()
            appendLine("import edu.wpi.first.wpilibj2.command.Command")
            appendLine()
            appendLine("interface $fileName {")
            appendLine()

            // per-entry functions with default implementations
            for (entry in entries) {
                appendLine("    fun ${entry.lowercase()}(): Command =")
                appendLine("        set${enumName}Value($enumName.$entry)")
                appendLine()
            }

            // abstract setter
            appendLine("    fun set${enumName}Value(value: $enumName): Command")
            appendLine("}")
        }

        file.write(classContent.toByteArray())
    }
}


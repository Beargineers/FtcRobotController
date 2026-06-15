package org.beargineers.platform

import fi.iki.elonen.NanoHTTPD
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream

object SettingsWebServer : NanoHTTPD(9000) {
    override fun serve(session: IHTTPSession): Response {
        val uri = session.uri

        return when {
            uri == "/" && session.method == Method.GET -> {
                serveSettingsForm(showSuccessMessage = false)
            }
            uri == "/" && session.method == Method.POST -> {
                handleSettingsUpdate(session)
            }
            uri == "/config" && session.method == Method.GET -> {
                handleConfigGet()
            }
            uri == "/config" && session.method == Method.POST -> {
                handleConfigPost(session)
            }
            uri == "/matchLogs" && session.method == Method.GET -> {
                handleMatchLogsGet()
            }
            else -> {
                newFixedLengthResponse(
                    Response.Status.NOT_FOUND,
                    "text/plain",
                    "Not found"
                )
            }
        }
    }

    private fun serveSettingsForm(showSuccessMessage: Boolean = false): Response {
        val successBanner = if (showSuccessMessage) {
            """
                <div id="success-banner" class="message">
                  <strong>Success!</strong> Settings have been saved.
                </div>
            """.trimIndent()
        } else {
            ""
        }

        val configText = escapeHtml(Config.currentConfigText)
        val html = """
            <!DOCTYPE html>
            <html>
              <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Settings Editor</title>
                <style>
                  body {
                    font-family: Arial, sans-serif;
                    max-width: 800px;
                    margin: 50px auto;
                    padding: 20px;
                  }
                  h1 {
                    color: #333;
                  }
                  textarea {
                    width: 100%;
                    min-height: 300px;
                    padding: 10px;
                    font-family: monospace;
                    font-size: 14px;
                    border: 1px solid #ccc;
                    border-radius: 4px;
                    box-sizing: border-box;
                  }
                  button {
                    background-color: #4CAF50;
                    color: white;
                    padding: 12px 24px;
                    border: none;
                    border-radius: 4px;
                    cursor: pointer;
                    font-size: 16px;
                    margin-top: 10px;
                  }
                  button:hover {
                    background-color: #45a049;
                  }
                  .message {
                    padding: 10px;
                    margin-bottom: 20px;
                    border-radius: 4px;
                    background-color: #d4edda;
                    color: #155724;
                    border: 1px solid #c3e6cb;
                  }
                  .warning {
                    padding: 10px;
                    margin-bottom: 20px;
                    border-radius: 4px;
                    background-color: #fff3cd;
                    color: #856404;
                    border: 1px solid #ffeeba;
                    display: none;
                  }
                </style>
                <script>
                  let originalSettings = `${configText.replace("`", "\\`")}`;

                  function checkForChanges() {
                    const textarea = document.getElementById('settings-textarea');
                    const warningBanner = document.getElementById('warning-banner');
                    const successBanner = document.getElementById('success-banner');

                    if (textarea.value !== originalSettings) {
                      warningBanner.style.display = 'block';
                      if (successBanner) {
                        successBanner.style.display = 'none';
                      }
                    } else {
                      warningBanner.style.display = 'none';
                    }
                  }

                  window.onload = function() {
                    const textarea = document.getElementById('settings-textarea');
                    textarea.addEventListener('input', checkForChanges);
                  };
                </script>
              </head>
              <body>
                <h1>Settings Editor</h1>
                $successBanner
                <div id="warning-banner" class="warning">
                  <strong>Warning!</strong> Settings have been changed. Press Save to apply changes.
                </div>
                <form method="POST" action="/">
                  <textarea id="settings-textarea" name="settings" placeholder="Enter settings here...">$configText</textarea>
                  <br>
                  <button type="submit">Save Settings</button>
                </form>
              </body>
            </html>
        """.trimIndent()

        return newFixedLengthResponse(
            Response.Status.OK,
            "text/html",
            html
        )
    }

    private fun handleSettingsUpdate(session: IHTTPSession): Response {
        // Parse POST data
        val files = HashMap<String, String>()
        try {
            session.parseBody(files)
        } catch (e: Exception) {
            return newFixedLengthResponse(
                Response.Status.INTERNAL_ERROR,
                "text/plain",
                "Error parsing form data: ${e.message}"
            )
        }

        // Get the settings value from the form
        Config.updateConfigAndSaveCache(session.parameters["settings"]?.firstOrNull() ?: "")

        // Respond with the settings form again, but with success message
        return serveSettingsForm(showSuccessMessage = true)
    }

    private fun escapeHtml(text: String): String {
        return text
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace("\"", "&quot;")
            .replace("'", "&#39;")
    }

    private fun handleConfigGet(): Response {
        return newFixedLengthResponse(
            Response.Status.OK,
            "text/plain",
            Config.currentConfigText
        )
    }

    private fun handleConfigPost(session: IHTTPSession): Response {
        // Read the request body
        val contentLength = session.headers["content-length"]?.toIntOrNull() ?: 0

        if (contentLength == 0) {
            return newFixedLengthResponse(
                Response.Status.BAD_REQUEST,
                "application/json",
                """{"status":"error","message":"Empty request body"}"""
            )
        }

        val body = ByteArray(contentLength)
        try {
            session.inputStream.read(body, 0, contentLength)
            val newConfigText = String(body, Charsets.UTF_8)

            // Update the config
            Config.updateConfigAndSaveCache(newConfigText)

            return newFixedLengthResponse(
                Response.Status.OK,
                "application/json",
                """{"status":"success","message":"Config updated successfully"}"""
            )
        } catch (e: Exception) {
            return newFixedLengthResponse(
                Response.Status.INTERNAL_ERROR,
                "application/json",
                """{"status":"error","message":"Error updating config: ${e.message}"}"""
            )
        }
    }

    private fun handleMatchLogsGet(): Response {
        val matchLogFolder = AppUtil.MATCH_LOG_FOLDER

        if (!matchLogFolder.exists()) {
            matchLogFolder.mkdirs()
        }

        val logs = matchLogFolder
            .walkTopDown()
            .filter { it.isFile }
            .toList()

        return try {
            val zipBytes = zipFiles(matchLogFolder, logs)
            val response = newFixedLengthResponse(
                Response.Status.OK,
                "application/zip",
                DeleteOnCompleteInputStream(zipBytes, logs),
                zipBytes.size.toLong()
            )

            response.addHeader("Content-Disposition", "attachment; filename=\"${matchLogsFileName()}\"")
            response.addHeader("Cache-Control", "no-store")
            response
        } catch (e: Exception) {
            newFixedLengthResponse(
                Response.Status.INTERNAL_ERROR,
                "text/plain",
                "Error creating match log zip: ${e.message}"
            )
        }
    }

    private fun zipFiles(rootFolder: File, files: List<File>): ByteArray {
        val byteStream = ByteArrayOutputStream()

        ZipOutputStream(byteStream).use { zipStream ->
            files.forEach { file ->
                val entryName = file
                    .relativeTo(rootFolder)
                    .invariantSeparatorsPath

                zipStream.putNextEntry(ZipEntry(entryName))
                file.inputStream().use { input ->
                    input.copyTo(zipStream)
                }
                zipStream.closeEntry()
            }
        }

        return byteStream.toByteArray()
    }

    private fun matchLogsFileName(): String {
        val timestamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
        return "match-logs-$timestamp.zip"
    }

    private class DeleteOnCompleteInputStream(
        bytes: ByteArray,
        private val filesToDelete: List<File>,
    ) : ByteArrayInputStream(bytes) {
        private var deleted = false

        override fun read(): Int {
            val value = super.read()
            deleteIfComplete()
            return value
        }

        override fun read(buffer: ByteArray, offset: Int, length: Int): Int {
            val count = super.read(buffer, offset, length)
            deleteIfComplete()
            return count
        }

        private fun deleteIfComplete() {
            if (available() == 0 && !deleted) {
                deleted = true
                filesToDelete.forEach { file ->
                    if (file.exists()) {
                        file.delete()
                    }
                }
            }
        }
    }
}

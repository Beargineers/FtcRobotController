package org.beargineers.platform

import fi.iki.elonen.NanoHTTPD

class SettingsWebServer(val robot: Robot, port: Int) : NanoHTTPD(port) {
    override fun serve(session: IHTTPSession): Response {
        val uri = session.uri

        return when {
            uri == "/" && session.method == Method.GET -> {
                serveSettingsForm(showSuccessMessage = false)
            }
            uri == "/" && session.method == Method.POST -> {
                handleSettingsUpdate(session)
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

        val configText = escapeHtml((robot as BaseRobot).currentConfigText)
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
        val newSettings = session.parameters["settings"]?.firstOrNull() ?: ""
        (robot as BaseRobot).updateConfigText(newSettings)

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
}
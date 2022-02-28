# Setup
## How to use example

### Preferences: Open Settings (JSON) -->settings.json
```json
"terminal.integrated.defaultProfile.windows": "Command Prompt",
    "terminal.integrated.profiles.windows": {
        "PowerShell": {
            "source": "PowerShell",
            "icon": "terminal-powershell"
        },
        "Command Prompt": {
            "path": [
                "${env:windir}\\Sysnative\\cmd.exe",
                "${env:windir}\\System32\\cmd.exe"
            ],
            "args": [
                "/k",
                "D:/Espressif/idf_cmd_init.bat esp-idf-ed32bcc5d69cd9888bb2dbbce369f6ce"
            ],
            "icon": "terminal-cmd"
        },
        "Git Bash": {
            "source": "Git Bash"
        }
    }
```

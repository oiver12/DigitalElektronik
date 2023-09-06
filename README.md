# Inhalt
- Alle Dateien in main Ordner, welche selbstprogrammiert sind \n
- Umgebung: ESP-IDF
# Setup (für VS Code)
-For compiling. Open in VS Code. Type into Terminal esp_idf. And compile with idf.py build. flash with idf.py flash.
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

### c_cpp_proprties.json
```json
  "name": "Win32",
            "includePath": [
                "${workspaceFolder}/**",
                "${workspaceFolder}/build/config",
                "D:/Espressif/frameworks/esp-idf-v4.4/components/**"
            ],
            "defines": [
                "_DEBUG",
                "UNICODE",
                "_UNICODE"
            ],
            "windowsSdkVersion": "10.0.19041.0",
            "compilerPath": "D:\\Espressif\\tools\\xtensa-esp32-elf\\esp-2021r2-patch2-8.4.0\\xtensa-esp32-elf\\bin\\xtensa-esp32-elf-gcc.exe",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "intelliSenseMode": "gcc-x64",
            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
```
### C/C++ Edit configurations (UI)
* Compiler path: esp../xtensa-esp32-elf-gcc.exe
* IntelliSense mode: gcc-x64
### launch.json
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "preLaunchTask": "opencdTask",
            "name": "ESP32 OpenOCD",
             "type": "cppdbg",
             "request": "launch",
             "cwd": "${workspaceFolder}/build",
             "program": "${workspaceFolder}/build/simple.elf",
             "miDebuggerPath": "D:/Espressif/tools/xtensa-esp32-elf/esp-2021r2-patch2-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gdb.exe",
             "setupCommands": [
                 {"text": "target remote 127.0.0.1:3333"},
                 {"text": "set remote hardware-watchpoint-limit 2"},
                 {"text": "monitor reset halt"},
                 {"text": "flushregs"},
                 {"text": "mon program_esp build/bootloader/bootloader.bin 0x1000 verify"},
                 {"text": "mon program_esp build/partition_table/partition-table.bin 0x8000 verify"},
                 {"text": "mon program_esp build/simple.bin 0x10000 verify"},
                 {"text": "monitor reset halt"},
                 {"text": "flushregs"}
             ]
        }
    ]
}
```
### task.json
```json
{
	"version": "2.0.0",
	"tasks": [
		{
			"label": "opencdTask",
			"type": "shell",
			"windows":{
				"command": "clear & start openocd -c \"set ESP_RTOS none\" -f board/esp32-wrover-kit-3.3v.cfg & exit"
			},
			"dependsOn":[
				"buildAlls"
			],
			"problemMatcher": []
		},
		{
			"label": "buildAlls",
			"type": "shell",
			"command": "D:/Espressif/Initialize-Idf.ps1 -IdfId esp-idf-ed32bcc5d69cd9888bb2dbbce369f6ce & start /wait python D:/Espressif/frameworks/esp-idf-v4.4/tools/idf.py build & exit",
			"problemMatcher": []
		}
	]
}
```

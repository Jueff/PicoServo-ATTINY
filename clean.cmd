if exist "%PLATFORMIO_WORKSPACE_DIR%\libdeps\PicoServo-Tiny" rd "%PLATFORMIO_WORKSPACE_DIR%\libdeps\PicoServo-Tiny\WS281XProcessor" /s/q
pio run --target clean


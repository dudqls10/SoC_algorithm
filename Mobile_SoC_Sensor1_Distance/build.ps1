$ErrorActionPreference = 'Stop'
$taskProjectRoot = $PSScriptRoot
$taskDebugDir = Join-Path $taskProjectRoot 'Debug'
[xml]$taskProjectXml = Get-Content -LiteralPath (Join-Path $taskProjectRoot '.project') -Raw
$taskProjectName = $taskProjectXml.projectDescription.name
$taskCompilerBin = 'C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.1.0.202410251130\tools\bin'
$taskMakeBin = 'C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin'
$taskPreviousPath = $env:PATH
$taskBase = Join-Path $taskDebugDir $taskProjectName
$taskPushed = $false
try {
    $env:PATH = $taskCompilerBin + ';' + $taskMakeBin + ';' + $taskPreviousPath
    Push-Location $taskDebugDir
    $taskPushed = $true
    # Rebuild all sources; never use objects from another project or old trial.
    & (Join-Path $taskMakeBin 'make.exe') -B -j4 all
    if ($LASTEXITCODE -ne 0) { throw ($taskProjectName + ' full build failed') }
    & (Join-Path $taskCompilerBin 'arm-none-eabi-objcopy.exe') -O binary ($taskBase + '.elf') ($taskBase + '.bin')
    if ($LASTEXITCODE -ne 0) { throw 'Binary export failed' }
    & (Join-Path $taskCompilerBin 'arm-none-eabi-objcopy.exe') -O ihex ($taskBase + '.elf') ($taskBase + '.hex')
    if ($LASTEXITCODE -ne 0) { throw 'Hex export failed' }
    Get-FileHash -LiteralPath ($taskBase + '.bin') -Algorithm SHA256
}
finally {
    if ($taskPushed) { Pop-Location }
    $env:PATH = $taskPreviousPath
}

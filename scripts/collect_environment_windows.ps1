$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")

function Capture-Command {
    param(
        [Parameter(Mandatory=$true)][string]$Name,
        [Parameter(Mandatory=$true)][string[]]$Arguments
    )

    try {
        $command = Get-Command $Name -ErrorAction Stop
        $output = & $command.Source @Arguments 2>&1 | Out-String
        return $output.Trim()
    }
    catch {
        return "NOT FOUND: $Name"
    }
}

$os = Get-CimInstance Win32_OperatingSystem
$cpu = Get-CimInstance Win32_Processor | Select-Object -First 1
$computer = Get-CimInstance Win32_ComputerSystem
$ramGiB = [math]::Round($computer.TotalPhysicalMemory / 1GB, 2)

$lines = @(
    "GRSI TESTED ENVIRONMENT",
    "=======================",
    "",
    "Generated: $([DateTime]::Now.ToString('yyyy-MM-dd HH:mm:ss K'))",
    "Repository commit: $(git rev-parse HEAD 2>$null)",
    "",
    "Operating system:",
    "$($os.Caption) $($os.Version) $($os.OSArchitecture)",
    "",
    "CPU:",
    "$($cpu.Name)",
    "",
    "Physical memory:",
    "$ramGiB GiB",
    "",
    "CMake:",
    (Capture-Command -Name "cmake" -Arguments @("--version")),
    "",
    "C++ compiler:",
    (Capture-Command -Name "g++" -Arguments @("--version")),
    "",
    "Build tool:",
    (Capture-Command -Name "mingw32-make" -Arguments @("--version")),
    "",
    "Python:",
    (Capture-Command -Name "python" -Arguments @("--version")),
    "",
    "Representative script:",
    "reproduce_eistute_windows.bat"
)

[System.IO.File]::WriteAllLines(
    (Join-Path (Get-Location) "TESTED_ENVIRONMENT.txt"),
    $lines,
    [System.Text.UTF8Encoding]::new($false)
)

Write-Host "Created TESTED_ENVIRONMENT.txt"

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Config,
    [Parameter(Mandatory = $true)]
    [string]$Results,
    [Parameter(Mandatory = $true)]
    [string]$Verifier,
    [string]$Executable = "build\bin\ap_mesh.exe"
)

$ErrorActionPreference = "Stop"
$repositoryRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $repositoryRoot

function Stop-WithStatus {
    param([string]$Message, [int]$Status)
    [Console]::Error.WriteLine($Message)
    exit $Status
}

$python = Get-Command python.exe -ErrorAction SilentlyContinue
if (-not $python) {
    Stop-WithStatus "Python was not found in PATH." 4
}
if (-not (Test-Path -LiteralPath $Config -PathType Leaf)) {
    Stop-WithStatus "Configuration file not found: $Config" 3
}
if (-not (Test-Path -LiteralPath $Verifier -PathType Leaf)) {
    Stop-WithStatus "Verifier not found: $Verifier" 5
}
if (-not (Test-Path -LiteralPath $Executable -PathType Leaf)) {
    Stop-WithStatus "Executable not found: $Executable" 4
}

& $python.Source scripts\validate_case_input.py $Config
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$resultsPath = [System.IO.Path]::GetFullPath((Join-Path $repositoryRoot $Results))
$repositoryPrefix = $repositoryRoot.TrimEnd(
    [System.IO.Path]::DirectorySeparatorChar,
    [System.IO.Path]::AltDirectorySeparatorChar
) + [System.IO.Path]::DirectorySeparatorChar
if (-not $resultsPath.StartsWith(
        $repositoryPrefix,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
    Stop-WithStatus "Results directory must stay inside the repository." 2
}
if (Test-Path -LiteralPath $resultsPath) {
    Remove-Item -LiteralPath $resultsPath -Recurse -Force
}
New-Item -ItemType Directory -Path $resultsPath | Out-Null

$runLog = Join-Path $resultsPath "run.log"
@(
    "Adaptive Non-Manifold Meshing"
    "Configuration: $Config"
    "Results directory: $Results"
    "Started: $([DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ'))"
    ""
) | Set-Content -LiteralPath $runLog -Encoding UTF8

& $Executable --config $Config 2>&1 | Tee-Object -FilePath $runLog -Append
$runStatus = $LASTEXITCODE
if ($runStatus -ne 0) {
    "Execution failed with status $runStatus." |
        Tee-Object -FilePath $runLog -Append |
        Write-Error
    exit $runStatus
}

"" | Tee-Object -FilePath $runLog -Append
"Verification: $Verifier" | Tee-Object -FilePath $runLog -Append
& $python.Source $Verifier $Results 2>&1 |
    Tee-Object -FilePath $runLog -Append
$verifyStatus = $LASTEXITCODE
if ($verifyStatus -ne 0) {
    exit $verifyStatus
}

"" | Tee-Object -FilePath $runLog -Append
"Completed: $([DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ'))" |
    Tee-Object -FilePath $runLog -Append

& $python.Source scripts\write_result_manifest.py $Results $Config
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
& $python.Source scripts\verify_result_manifest.py $Results
exit $LASTEXITCODE

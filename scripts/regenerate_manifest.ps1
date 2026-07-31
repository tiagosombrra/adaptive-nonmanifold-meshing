$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")

$manifestPath = Join-Path (Get-Location) "MANIFEST.sha256"
Remove-Item -LiteralPath $manifestPath -Force -ErrorAction SilentlyContinue

$paths = git ls-files -co --exclude-standard |
    Where-Object { $_ -and $_ -ne "MANIFEST.sha256" } |
    Sort-Object -Unique

$lines = foreach ($path in $paths) {
    $hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $path).Hash.ToLowerInvariant()
    "$hash *./$path"
}

[System.IO.File]::WriteAllLines(
    $manifestPath,
    $lines,
    [System.Text.Encoding]::ASCII
)

$errors = 0
foreach ($line in Get-Content -LiteralPath $manifestPath) {
    if ($line -notmatch '^([0-9a-fA-F]{64}) \*\./(.+)$') {
        Write-Host "Malformed line: $line"
        $errors++
        continue
    }

    $expected = $matches[1].ToLowerInvariant()
    $path = $matches[2]

    if (-not (Test-Path -LiteralPath $path)) {
        Write-Host "${path}: MISSING"
        $errors++
        continue
    }

    $actual = (Get-FileHash -Algorithm SHA256 -LiteralPath $path).Hash.ToLowerInvariant()
    if ($actual -eq $expected) {
        Write-Host "${path}: OK"
    }
    else {
        Write-Host "${path}: FAILED"
        $errors++
    }
}

if ($errors -gt 0) {
    throw "Manifest validation failed with $errors problem(s)."
}

Write-Host "MANIFEST.sha256 regenerated and validated."

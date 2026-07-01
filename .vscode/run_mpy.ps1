param(
    [Parameter(Mandatory)]
    [string]$Action,

    [string]$FilePath,
    [string]$RelativePath
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# ---------- detect port ----------
$port = python "$scriptDir/detect_port.py"
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: no board found (VID:PID=F055:9802)" -ForegroundColor Red
    exit 1
}
$p = $port.Trim()
Write-Host "Connected - $p" -ForegroundColor Green

# ---------- execute action ----------
switch ($Action) {
    "run" {
        python .vscode/mpy_run.py connect $p resume run $FilePath
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Run OK" -ForegroundColor Green
        } else {
            Write-Host "Error: run failed" -ForegroundColor Red
            exit 1
        }
    }
    "upload" {
        python .vscode/mpy_run.py connect $p resume fs cp $FilePath ":$RelativePath"
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Upload OK" -ForegroundColor Green
        } else {
            Write-Host "Error: upload failed" -ForegroundColor Red
            exit 1
        }
    }
    "repl" {
        Write-Host "Entering REPL, press Ctrl+] to exit" -ForegroundColor Cyan
        python .vscode/mpy_run.py connect $p resume repl
    }
    "reset" {
        $ErrorActionPreference = "Continue"
        python .vscode/mpy_run.py connect $p resume exec "import machine; machine.reset()" 2>$null
        $ErrorActionPreference = "Stop"
        Write-Host "Reset OK, board rebooting..." -ForegroundColor Green
    }
    "ls" {
        python .vscode/mpy_run.py connect $p resume fs ls
        if ($LASTEXITCODE -eq 0) {
            Write-Host "List OK" -ForegroundColor Green
        } else {
            Write-Host "Error: list failed" -ForegroundColor Red
            exit 1
        }
    }
    "cp-main" {
        python .vscode/mpy_run.py connect $p resume fs cp main.py :main.py
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Upload OK (main.py -> /flash/main.py)" -ForegroundColor Green
        } else {
            Write-Host "Error: upload failed" -ForegroundColor Red
            exit 1
        }
    }
    "sync" {
        $projectDir = Split-Path -Parent $scriptDir
        $pyFiles = Get-ChildItem -Path $projectDir -Filter "*.py" -File
        $total = $pyFiles.Count
        $ok = 0
        foreach ($f in $pyFiles) {
            $remote = ":" + $f.Name
            python .vscode/mpy_run.py connect $p resume fs cp $f.FullName $remote
            if ($LASTEXITCODE -eq 0) {
                Write-Host "  [$($ok+1)/$total] $($f.Name)" -ForegroundColor Green
                $ok++
            } else {
                Write-Host "  FAIL: $($f.Name)" -ForegroundColor Red
            }
        }
        Write-Host "Sync done: $ok/$total files uploaded" -ForegroundColor Green
    }
    "deploy" {
        # Step 1: upload file to flash
        python .vscode/mpy_run.py connect $p resume fs cp $FilePath ":$RelativePath"
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Error: upload failed" -ForegroundColor Red
            exit 1
        }
        Write-Host "Upload OK" -ForegroundColor Green
        # Step 2: reset board to run the new code from flash
        $ErrorActionPreference = "Continue"
        python .vscode/mpy_run.py connect $p resume exec "import machine; machine.reset()" 2>$null
        $ErrorActionPreference = "Stop"
        Write-Host "Reset OK, board rebooting..." -ForegroundColor Green
    }
    default {
        Write-Host "Error: unknown action '$Action'" -ForegroundColor Red
        exit 1
    }
}

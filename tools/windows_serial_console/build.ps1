$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$outDir = Join-Path $scriptDir "bin"
$exePath = Join-Path $outDir "H7TofSerialConsole.exe"
$csc = "C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe"

if (-not (Test-Path -LiteralPath $csc)) {
    throw "csc.exe not found: $csc"
}

New-Item -ItemType Directory -Force -Path $outDir | Out-Null

& $csc /nologo /target:winexe /optimize+ /out:$exePath /r:System.dll /r:System.Drawing.dll /r:System.Windows.Forms.dll /r:System.Runtime.Serialization.dll (Join-Path $scriptDir "Program.cs")
if ($LASTEXITCODE -ne 0) {
    throw "Desktop build failed with exit code $LASTEXITCODE"
}

Write-Host "Built desktop app:" $exePath

param(
    [Parameter(Mandatory = $true)]
    [string]$LaneDir,

    [Parameter(Mandatory = $true)]
    [string]$Command,

    [Parameter(Mandatory = $true)]
    [string]$EnvironmentName,

    [Parameter(Mandatory = $true)]
    [string]$ToolVersion,

    [Parameter(Mandatory = $true)]
    [string]$ResultLabel,

    [string[]]$ArtifactPaths = @(),

    [string]$Reason = ""
)

$ErrorActionPreference = "Stop"

New-Item -ItemType Directory -Force -Path $LaneDir | Out-Null

$commandPath = Join-Path $LaneDir "command.txt"
$stdoutPath = Join-Path $LaneDir "stdout.log"
$stderrPath = Join-Path $LaneDir "stderr.log"
$exitCodePath = Join-Path $LaneDir "exit-code.txt"
$summaryPath = Join-Path $LaneDir "summary.json"

Set-Content -LiteralPath $commandPath -Value $Command

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = "powershell.exe"
$psi.Arguments = "-NoProfile -ExecutionPolicy Bypass -Command `$ErrorActionPreference = 'Stop'; $Command"
$psi.WorkingDirectory = (Get-Location).Path
$psi.UseShellExecute = $false
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true

$process = New-Object System.Diagnostics.Process
$process.StartInfo = $psi

$null = $process.Start()
$stdout = $process.StandardOutput.ReadToEnd()
$stderr = $process.StandardError.ReadToEnd()
$process.WaitForExit()
$exitCode = $process.ExitCode

Set-Content -LiteralPath $stdoutPath -Value $stdout
Set-Content -LiteralPath $stderrPath -Value $stderr
Set-Content -LiteralPath $exitCodePath -Value ([string]$exitCode)

$summary = [ordered]@{
    timestamp = (Get-Date).ToString("o")
    environment = $EnvironmentName
    tool_version = $ToolVersion
    command = $Command
    exit_code = $exitCode
    result = $ResultLabel
    artifact_paths = $ArtifactPaths
}

if ($Reason -ne "") {
    $summary.reason = $Reason
}

$summary | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $summaryPath

exit $exitCode

param(
    [Parameter(Mandatory = $true)]
    [string]$LaneDir,

    [Parameter(Mandatory = $true)]
    [string]$EnvironmentName,

    [Parameter(Mandatory = $true)]
    [string]$ToolVersion,

    [Parameter(Mandatory = $true)]
    [string]$Command,

    [Parameter(Mandatory = $true)]
    [int]$ExitCode,

    [Parameter(Mandatory = $true)]
    [string]$Result,

    [string[]]$ArtifactPaths = @(),

    [string]$Reason = ""
)

$ErrorActionPreference = "Stop"

$summaryPath = Join-Path $LaneDir "summary.json"
$summary = [ordered]@{
    timestamp = (Get-Date).ToString("o")
    environment = $EnvironmentName
    tool_version = $ToolVersion
    command = $Command
    exit_code = $ExitCode
    result = $Result
    artifact_paths = $ArtifactPaths
}

if ($Reason -ne "") {
    $summary.reason = $Reason
}

$summary | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $summaryPath

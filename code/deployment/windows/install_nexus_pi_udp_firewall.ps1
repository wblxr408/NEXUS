#Requires -RunAsAdministrator
$ErrorActionPreference = "Stop"
$ruleName = "NEXUS Pi Telemetry UDP 14551"
if (-not (Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Action Allow `
        -Protocol UDP -LocalPort 14551 -RemoteAddress 192.168.1.143 -Profile Private
}
Get-NetFirewallRule -DisplayName $ruleName |
    Select-Object DisplayName, Enabled, Direction, Action

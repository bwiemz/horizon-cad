# Installs the Windows installer of Horizon CAD as a user would, on a machine
# that never built it, starts it with --self-test, and uninstalls it (see
# packaging/linux/check-install.sh and .github/workflows/install-check.yml).
# --self-test models, edits, saves and reads back a part, an assembly and a
# drawing, opens the samples, loads the translations, and shows the window.
#
#   packaging/windows/check-install.ps1 -Installer <HorizonCAD-...-win64.exe>
param([Parameter(Mandatory = $true)][string] $Installer)
$ErrorActionPreference = 'Stop'

function Wait-Exit([System.Diagnostics.Process] $Process, [int] $Seconds, [string] $What) {
    if (-not $Process.WaitForExit($Seconds * 1000)) {
        $Process.Kill()
        throw "$What did not finish within $Seconds s"
    }
    return $Process.ExitCode
}

# Installed silently, where it goes when its user accepts what it offers.
$process = Start-Process -FilePath $Installer -ArgumentList '/S' -PassThru
$null = $process.Handle  # without it, ExitCode is not kept
$status = Wait-Exit $process 300 'The installer'
if ($status -ne 0) { throw "The installer exited $status" }
$dir = Join-Path $env:ProgramFiles 'HorizonCAD'
$exe = Join-Path $dir 'bin\horizon.exe'
if (-not (Test-Path $exe)) { throw "Nothing was installed at $exe" }
Write-Output "Installed at $dir"

# Its shortcut, on the desktop of whoever it was installed for.
$shortcuts = @(
    (Join-Path ([Environment]::GetFolderPath('CommonDesktopDirectory')) 'Horizon CAD.lnk'),
    (Join-Path ([Environment]::GetFolderPath('Desktop')) 'Horizon CAD.lnk')
) | Where-Object { Test-Path $_ }
if ($shortcuts.Count -eq 0) { throw 'No "Horizon CAD" shortcut on the desktop' }
$target = (New-Object -ComObject WScript.Shell).CreateShortcut($shortcuts[0]).TargetPath
if ($target -ne $exe) { throw "The shortcut starts $target, not $exe" }

# A program of the Windows subsystem has no console: what it says is kept
# only when its output is redirected.
$out = Join-Path $env:RUNNER_TEMP 'self-test.out'
$err = Join-Path $env:RUNNER_TEMP 'self-test.err'
$process = Start-Process -FilePath $exe -ArgumentList '--self-test' -PassThru `
    -RedirectStandardOutput $out -RedirectStandardError $err
$null = $process.Handle
$status = Wait-Exit $process 180 'horizon --self-test'
Get-Content $out, $err
switch ($status) {
    0 { Write-Output 'The self-test passed' }
    # A runner has no graphics card; everything else passed (5 says otherwise).
    3 { Write-Output '::warning::the application started, but its viewport could not draw on this runner' }
    # 0xC0000135: a DLL it needs is not there.
    -1073741515 { throw 'horizon.exe cannot start: a DLL it needs is not installed (0xC0000135)' }
    default { throw "horizon --self-test exited $status" }
}

# Uninstalled silently: nothing of it is left where it was. The uninstaller
# runs where it is (_?=), so it can be waited for, and cannot remove itself.
$uninstaller = Join-Path $dir 'Uninstall.exe'
$process = Start-Process -FilePath $uninstaller -ArgumentList '/S', "_?=$dir" -PassThru
$null = $process.Handle
$status = Wait-Exit $process 300 'The uninstaller'
if ($status -ne 0) { throw "The uninstaller exited $status" }
if (Test-Path $exe) { throw "$exe is still there after uninstalling" }
$left = @(Get-ChildItem -Recurse -File $dir | Where-Object { $_.Name -ne 'Uninstall.exe' })
if ($left.Count -gt 0) { throw "Left behind after uninstalling: $($left.FullName -join ', ')" }
$shortcuts = @($shortcuts | Where-Object { Test-Path $_ })
if ($shortcuts.Count -gt 0) { throw "The shortcut is still there: $($shortcuts -join ', ')" }
Write-Output 'Uninstalled'

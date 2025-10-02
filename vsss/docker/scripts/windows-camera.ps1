param(
    [Parameter(Mandatory=$true)]
    [ValidateSet("attach","detach")]
    [string]$Action
)

# Get the usbipd device list
$devices = usbipd list

# Find the first device that has "Camera" in its description
$camera = $devices | Select-String "Camera" | Select-Object -First 1

if ($null -eq $camera) {
    Write-Host "❌ No camera device found in usbipd list."
    exit 1
}

# Extract BusID (first column)
$busid = $camera.ToString().Split(" ", [System.StringSplitOptions]::RemoveEmptyEntries)[0]

switch ($Action) {
    "attach" {
        Write-Host "📷 Attaching camera at BusID $busid to WSL..."
        usbipd attach --wsl --busid $busid
        Write-Host "✅ Camera attached to WSL (not available in Windows now)."
    }
    "detach" {
        Write-Host "🔌 Detaching camera at BusID $busid from WSL..."
        usbipd detach --busid $busid
        Write-Host "✅ Camera detached (back available in Windows)."
    }
}

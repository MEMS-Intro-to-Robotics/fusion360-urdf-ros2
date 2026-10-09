try {
    # Paths are relative to this script, so it works from any working directory
    $source = Join-Path $PSScriptRoot "Fusion_URDF_Exporter_ROS2"
    $scripts = "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts"
    $destination = Join-Path $scripts "Fusion_URDF_Exporter_ROS2"
    $staging = "$destination.new"

    Write-Host "Starting installation process..." -ForegroundColor Green

    if (-not (Test-Path -Path $source)) {
        throw "Cannot find $source. Keep this script next to the Fusion_URDF_Exporter_ROS2 folder."
    }
    New-Item -ItemType Directory -Force -Path $scripts | Out-Null

    # Copy to a staging folder first, so a failed copy leaves the existing install in place
    if (Test-Path -Path $staging) {
        Remove-Item -Path $staging -Recurse -Force
    }
    Copy-Item -Path $source -Destination $staging -Recurse

    # Replace the existing folder
    if (Test-Path -Path $destination) {
        Write-Host "Existing folder found. Replacing..." -ForegroundColor Yellow
        Remove-Item -Path $destination -Recurse -Force
    }
    Rename-Item -Path $staging -NewName "Fusion_URDF_Exporter_ROS2"
    Write-Host "Installation completed successfully!" -ForegroundColor Green
} catch {
    # Print the error if something goes wrong
    Write-Host "An error occurred during installation: $($_.Exception.Message)" -ForegroundColor Red
}

# Exit immediately
exit

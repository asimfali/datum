param(
  [string]$Dir,
  [string]$WslDir,
  [int]$IntervalMs = 500,
  [switch]$Once
)

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

if (-not $Dir) { $Dir = Join-Path $env:LOCALAPPDATA 'Temp\opencode-shots' }
if (-not $WslDir) { $WslDir = $Dir -replace '^([A-Za-z]):', '/mnt/$1' -replace '\\', '/' }

New-Item -ItemType Directory -Force -Path $Dir | Out-Null
Get-ChildItem $Dir -Filter 'shot-*.png' -ErrorAction SilentlyContinue |
  Where-Object { $_.LastWriteTime -lt (Get-Date).AddDays(-7) } |
  Remove-Item -Force -ErrorAction SilentlyContinue

function Get-Signature($image) {
  $bmp = New-Object System.Drawing.Bitmap 8, 8
  $g = [System.Drawing.Graphics]::FromImage($bmp)
  $g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBilinear
  $g.DrawImage($image, 0, 0, 8, 8)
  $g.Dispose()
  $sb = New-Object System.Text.StringBuilder
  for ($y = 0; $y -lt 8; $y++) {
    for ($x = 0; $x -lt 8; $x++) {
      [void]$sb.Append($bmp.GetPixel($x, $y).ToArgb())
    }
  }
  $bmp.Dispose()
  return "$($image.Width)x$($image.Height):$($sb.ToString().GetHashCode())"
}

$last = ''
while ($true) {
  try {
    if (-not [System.Windows.Forms.Clipboard]::ContainsImage()) {
      $last = ''
    } else {
      $image = [System.Windows.Forms.Clipboard]::GetImage()
      if ($null -ne $image) {
        $sig = Get-Signature $image
        if ($sig -ne $last) {
          $name = 'shot-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.png'
          $image.Save((Join-Path $Dir $name), [System.Drawing.Imaging.ImageFormat]::Png)
          $data = New-Object System.Windows.Forms.DataObject
          $data.SetImage($image)
          $data.SetText("$WslDir/$name")
          [System.Windows.Forms.Clipboard]::SetDataObject($data, $true)
          $last = $sig
        }
      }
    }
  } catch {
    # clipboard busy or an image we cannot decode; try again next tick
  }
  if ($Once) { break }
  Start-Sleep -Milliseconds $IntervalMs
}

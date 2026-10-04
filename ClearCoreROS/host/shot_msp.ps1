param([Parameter(Mandatory=$true)][string]$Path)
Add-Type @"
using System;
using System.Text;
using System.Runtime.InteropServices;
public class MspShot {
  public delegate bool EnumProc(IntPtr hWnd, IntPtr lParam);
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc lp, IntPtr l);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  public struct RECT { public int L, T, R, B; }
  public static IntPtr Find() {
    IntPtr found = IntPtr.Zero;
    EnumWindows((h, l) => {
      var title = new StringBuilder(256);
      GetWindowText(h, title, 256);
      if (IsWindowVisible(h) && title.ToString().StartsWith("ClearPath-MSP")) { found = h; return false; }
      return true;
    }, IntPtr.Zero);
    return found;
  }
}
"@
Add-Type -AssemblyName System.Drawing
$h = [MspShot]::Find()
if ($h -eq [IntPtr]::Zero) { throw "ClearPath-MSP window not found" }
$r = New-Object MspShot+RECT
[void][MspShot]::GetWindowRect($h, [ref]$r)
$w = $r.R - $r.L; $hh = $r.B - $r.T
if ($w -lt 50) { throw "MSP window has no size" }
$bmp = New-Object System.Drawing.Bitmap $w, $hh
$g = [System.Drawing.Graphics]::FromImage($bmp)
$hdc = $g.GetHdc()
[void][MspShot]::PrintWindow($h, $hdc, 2)
$g.ReleaseHdc($hdc)
$bmp.Save($Path, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
Write-Output "saved $Path ${w}x${hh}"

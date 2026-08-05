# 全向位置控制器主机侧测试构建脚本（Windows）
# 用法：powershell -ExecutionPolicy Bypass -File build_and_run.ps1
$ErrorActionPreference = 'Stop'

$gcc = 'D:\msys64\ucrt64\bin\gcc.exe'
if (-not (Test-Path -LiteralPath $gcc)) {
  $cmd = Get-Command gcc -ErrorAction SilentlyContinue
  if ($null -ne $cmd) { $gcc = $cmd.Source }
}
if (-not (Test-Path -LiteralPath $gcc)) {
  Write-Error '未找到主机 gcc（预期 D:\msys64\ucrt64\bin\gcc.exe 或 PATH 中的 gcc）'
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoDir = Split-Path -Parent (Split-Path -Parent $scriptDir)
$outExe = Join-Path $env:TEMP 'holonomic_profile_test.exe'

& $gcc -std=c99 -Wall -DADVANCE_HOLONOMIC_UNIT_TEST `
  -I (Join-Path $scriptDir 'stubs') `
  -I (Join-Path $repoDir 'Core/Inc') `
  (Join-Path $scriptDir 'test_holonomic_profile.c') `
  -lm -o $outExe
if ($LASTEXITCODE -ne 0) {
  Write-Error "编译失败，退出码 $LASTEXITCODE"
}

& $outExe
$testExit = $LASTEXITCODE
Remove-Item -LiteralPath $outExe -ErrorAction SilentlyContinue
exit $testExit

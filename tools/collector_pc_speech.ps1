#requires -Version 5.1
[CmdletBinding()]
param(
    [string]$Robot = '',
    [string]$Repo = '/home/ubuntu/CB_Autolife_Data_Collector',
    [string]$BaseDir = '',
    [switch]$SelfTest
)
$ErrorActionPreference = 'Stop'
$script:lastSpeechId = ''

function Read-SpeechPacket([string]$Line) {
    try { $item = $Line | ConvertFrom-Json -ErrorAction Stop }
    catch { throw '播报连接收到非协议数据，请检查SSH连接。' }
    if ($null -eq $item -or -not ($item.kind -is [string])) { throw '无效的播报消息。' }
    if ($item.kind -eq 'speech') {
        if (-not ($item.id -is [string]) -or -not $item.id -or
            -not ($item.text -is [string]) -or -not $item.text) { throw '播报消息缺少事件编号或文字。' }
        if ($item.id -eq $script:lastSpeechId) { return $null }
        $script:lastSpeechId = $item.id
    }
    return $item
}

function New-SpeechCommand([string]$Repository, [string]$Directory) {
    # The SSH command contains a validated script path and base64 data only.
    if ($Repository -notmatch '^/[A-Za-z0-9_./-]+$' -or $Repository -match '/\.\./') {
        throw '仓库路径须为不含空格的Linux绝对路径。'
    }
    $command = 'python3 -u ' + $Repository.TrimEnd('/') + '/collector_pc_speech.py'
    if ($Directory) {
        if (-not $Directory.StartsWith('/')) { throw '数据目录必须是机器人上的绝对路径。' }
        $encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Directory))
        $command += ' --base-dir-b64 ' + $encoded
    }
    return $command
}

if ($SelfTest) {
    $spoken = @()
    foreach ($number in 1..8) {
        $line = @{kind='speech'; id="log:$number"; text='下一步'} | ConvertTo-Json -Compress
        $item = Read-SpeechPacket $line
        if ($null -ne $item) { $spoken += $item.text }
    }
    if ($spoken.Count -ne 8) { throw '连续相同文本漏播' }
    if ($null -ne (Read-SpeechPacket $line)) { throw '重复事件未去重' }
    foreach ($number in 39,40) {
        $item = Read-SpeechPacket (@{kind='speech'; id="saved:$number"; text="第${number}条已保存"} | ConvertTo-Json -Compress)
        if ($null -eq $item) { throw '不同条目的保存反馈被合并' }
    }
    $script:lastSpeechId = ''
    $path = '/home/ubuntu/nas/中文目录; echo unexpected'
    $command = New-SpeechCommand '/home/ubuntu/CB_Autolife_Data_Collector' $path
    if ($command.Contains(';') -or $command.Contains('unexpected')) { throw '路径未安全编码' }
    $decoded = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String(($command -split ' ')[-1]))
    if ($decoded -ne $path) { throw '路径编码错误' }
    $rejected = $false
    try { New-SpeechCommand '/repo;echo' '' | Out-Null } catch { $rejected = $true }
    if (-not $rejected) { throw '非法仓库路径未拒绝' }
    $rejected = $false
    try { Read-SpeechPacket '{"kind":"speech","id":1}' | Out-Null } catch { $rejected = $true }
    if (-not $rejected) { throw '错误消息未拒绝' }
    Write-Host 'PASS: 8 repeated marks, distinct saves, event identity, UTF-8 paths and invalid input.'
    exit 0
}

$synth = $null
$oldOutputEncoding = $OutputEncoding
$oldConsoleEncoding = [Console]::OutputEncoding
try {
    if ([Environment]::OSVersion.Platform -ne 'Win32NT') { throw '播放声音需要Windows；逻辑自测可用 -SelfTest。' }
    if (-not $Robot) {
        $Robot = Read-Host '机器人IP或SSH别名（直接回车为340：192.168.8.59）'
        if (-not $Robot) { $Robot = '192.168.8.59' }
    }
    if ($Robot -notmatch '^[A-Za-z0-9_][A-Za-z0-9_.@:-]*$') { throw '机器人地址格式错误。' }
    if (-not $Robot.Contains('@')) { $Robot = 'ubuntu@' + $Robot }
    $remoteCommand = New-SpeechCommand $Repo $BaseDir
    $ssh = Get-Command ssh.exe -ErrorAction Stop
    Add-Type -AssemblyName System.Speech
    $synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
    $voices = @($synth.GetInstalledVoices() | Where-Object { $_.Enabled -and $_.VoiceInfo.Culture.Name -like 'zh-*' })
    if ($voices.Count -eq 0) { throw 'Windows没有可用中文语音，请先在语言设置添加中文语音。' }
    $synth.SelectVoice($voices[0].VoiceInfo.Name)
    $synth.SetOutputToDefaultAudioDevice()
    $synth.Rate = 1
    $synth.Volume = 90
    $utf8 = New-Object Text.UTF8Encoding($false)
    $OutputEncoding = $utf8
    [Console]::OutputEncoding = $utf8
    Write-Host '云蝶采集电脑播报 | 只读连接，不控制机器人'
    Write-Host '先打开采集服务，再连接播报。Ctrl+C只停止播报；退出采集仍在原终端按Q。'
    Write-Host '首次连接按SSH提示核对主机，密码由SSH读取，本工具不保存密码。'
    $sshArgs = @('-T', '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3', $Robot, $remoteCommand)
    $script:connected = $false
    $script:ended = $false
    # Each SSH output line is an event, not a screen snapshot or text keyword.
    & $ssh.Source @sshArgs | ForEach-Object {
        $item = Read-SpeechPacket ([string]$_)
        if ($null -eq $item) { return }
        switch ($item.kind) {
            'connected' {
                $script:connected = $true
                Write-Host ('已连接：' + $item.base_dir) -ForegroundColor Green
                Write-Host ('来源：' + $item.source)
                [void]$synth.SpeakAsync('电脑播报已连接')
            }
            'speech' {
                Write-Host ('[采集] ' + $item.text)
                [void]$synth.SpeakAsync([string]$item.text)
            }
            'heartbeat' { }
            'ended' {
                $script:ended = $true
                Write-Host '状态来源已退出，请检查原采集终端的最终结果。' -ForegroundColor Yellow
                [void]$synth.SpeakAsync('状态来源已退出，请检查采集终端')
            }
            'error' { throw ('播报连接失败：' + $item.text) }
            default { throw '不支持的播报协议，请核对机器人端与电脑端版本。' }
        }
    }
    if ($LASTEXITCODE -ne 0 -or -not $script:ended) {
        throw '播报连接已断开，采集状态未知；请检查采集终端后重新连接。'
    }
    # Drain acknowledged status cues on normal EOF. Ctrl+C still cancels promptly.
    while ($synth.State -ne 'Ready') { Start-Sleep -Milliseconds 100 }
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    if ($null -ne $synth) {
        $synth.SpeakAsyncCancelAll()
        $synth.Speak('电脑播报连接异常，请检查终端')
    }
    exit 1
} finally {
    if ($null -ne $synth) { $synth.SpeakAsyncCancelAll(); $synth.Dispose() }
    $OutputEncoding = $oldOutputEncoding
    [Console]::OutputEncoding = $oldConsoleEncoding
}

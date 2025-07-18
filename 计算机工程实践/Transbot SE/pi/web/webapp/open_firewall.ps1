# 以管理员权限添加防火墙规则允许Flask 5000端口
New-NetFirewallRule -Name "Flask 5000" -DisplayName "Flask Web App" -Direction Inbound -Protocol TCP -LocalPort 5000 -Action Allow

Write-Host "防火墙规则已添加，允许5000端口入站流量" -ForegroundColor Green
Write-Host "现在局域网中的其他设备应该可以通过 http://192.168.144.17:5000/ 访问您的应用了" -ForegroundColor Cyan

Read-Host -Prompt "按Enter键退出" 
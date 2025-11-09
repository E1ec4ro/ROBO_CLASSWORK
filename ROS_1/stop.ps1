# PowerShell скрипт для остановки ROS Docker контейнера

Write-Host "Остановка ROS Docker контейнера..." -ForegroundColor Yellow
docker-compose down

Write-Host ""
Write-Host "✅ Контейнер остановлен!" -ForegroundColor Green


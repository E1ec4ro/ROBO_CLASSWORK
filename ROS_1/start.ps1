# PowerShell скрипт для запуска ROS Docker контейнера

Write-Host "=== ROS Docker Environment ===" -ForegroundColor Cyan
Write-Host ""

# Проверка наличия Docker
if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Host "Ошибка: Docker не установлен!" -ForegroundColor Red
    exit 1
}

# Проверка наличия Docker Compose
if (-not (Get-Command docker-compose -ErrorAction SilentlyContinue)) {
    Write-Host "Ошибка: Docker Compose не установлен!" -ForegroundColor Red
    exit 1
}

# Создание необходимых директорий
Write-Host "Создание рабочих директорий..." -ForegroundColor Yellow
New-Item -ItemType Directory -Force -Path "workspace" | Out-Null
New-Item -ItemType Directory -Force -Path "config" | Out-Null

# Сборка образа (если нужно)
Write-Host ""
$rebuild = Read-Host "Пересобрать образ? (y/n)"
if ($rebuild -eq "y" -or $rebuild -eq "Y") {
    Write-Host "Сборка Docker образа..." -ForegroundColor Yellow
    docker-compose build
}

# Запуск контейнера
Write-Host ""
Write-Host "Запуск контейнера..." -ForegroundColor Yellow
docker-compose up -d

# Ожидание запуска
Write-Host ""
Write-Host "Ожидание запуска сервисов..." -ForegroundColor Yellow
Start-Sleep -Seconds 5

# Проверка статуса
$container = docker ps --filter "name=ros_gazebo_rviz" --format "{{.Names}}"
if ($container -eq "ros_gazebo_rviz") {
    Write-Host ""
    Write-Host "✅ Контейнер успешно запущен!" -ForegroundColor Green
    Write-Host ""
    Write-Host "🌐 Откройте браузер и перейдите по адресу:" -ForegroundColor Cyan
    Write-Host "   http://localhost:6080/vnc.html" -ForegroundColor White
    Write-Host ""
    Write-Host "📝 Для доступа к терминалу контейнера:" -ForegroundColor Cyan
    Write-Host "   docker exec -it ros_gazebo_rviz bash" -ForegroundColor White
    Write-Host ""
    Write-Host "🛑 Для остановки контейнера:" -ForegroundColor Cyan
    Write-Host "   docker-compose down" -ForegroundColor White
} else {
    Write-Host ""
    Write-Host "❌ Ошибка при запуске контейнера!" -ForegroundColor Red
    Write-Host "Проверьте логи: docker-compose logs" -ForegroundColor Yellow
}


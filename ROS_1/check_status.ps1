# PowerShell скрипт для проверки статуса сервисов в контейнере

Write-Host "=== Проверка статуса контейнера ===" -ForegroundColor Cyan
docker ps | Select-String "ros_gazebo_rviz"

Write-Host ""
Write-Host "=== Проверка логов supervisor ===" -ForegroundColor Cyan
docker exec ros_gazebo_rviz tail -n 20 /var/log/supervisor/supervisord.log

Write-Host ""
Write-Host "=== Проверка статуса процессов ===" -ForegroundColor Cyan
docker exec ros_gazebo_rviz supervisorctl status

Write-Host ""
Write-Host "=== Проверка портов ===" -ForegroundColor Cyan
docker exec ros_gazebo_rviz netstat -tlnp 2>$null | Select-String -Pattern "6080|5900"

Write-Host ""
Write-Host "=== Проверка Xvfb ===" -ForegroundColor Cyan
docker exec ros_gazebo_rviz ps aux | Select-String "Xvfb"

Write-Host ""
Write-Host "=== Проверка x11vnc ===" -ForegroundColor Cyan
docker exec ros_gazebo_rviz ps aux | Select-String "x11vnc"

Write-Host ""
Write-Host "=== Проверка websockify ===" -ForegroundColor Cyan
docker exec ros_gazebo_rviz ps aux | Select-String "websockify"

Write-Host ""
Write-Host "=== Проверка noVNC файлов ===" -ForegroundColor Cyan
docker exec ros_gazebo_rviz ls -la /opt/novnc/ | Select-Object -First 10


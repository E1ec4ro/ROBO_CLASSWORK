# Используем официальный образ ROS Noetic с полным десктопом
FROM osrf/ros:noetic-desktop-full

# Установка переменных окружения
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:1
ENV ROS_DISTRO=noetic

# Установка необходимых пакетов
RUN apt-get update && apt-get install -y \
    # Базовые инструменты
    wget \
    curl \
    git \
    vim \
    nano \
    # X11 и VNC для веб-интерфейса
    xfce4 \
    xfce4-goodies \
    x11vnc \
    xvfb \
    fluxbox \
    # noVNC для веб-доступа
    novnc \
    websockify \
    # Дополнительные инструменты для ROS
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-rviz \
    # Python инструменты
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    # Зависимости для сборки Python пакетов
    python3-dev \
    libffi-dev \
    libssl-dev \
    # Дополнительные утилиты
    terminator \
    dbus-x11 \
    supervisor \
    # Создание директорий для логов
    && mkdir -p /var/log/supervisor \
    && rm -rf /var/lib/apt/lists/*

# Инициализация rosdep
RUN rosdep init || true

# Обновление pip
RUN python3 -m pip install --upgrade pip setuptools wheel

# Установка noVNC из GitHub (удаляем существующую директорию, если она есть)
RUN rm -rf /opt/novnc && \
    git clone https://github.com/novnc/noVNC.git /opt/novnc && \
    git clone https://github.com/novnc/websockify.git /opt/novnc/utils/websockify && \
    cd /opt/novnc/utils/websockify && \
    pip3 install .

# Создание конфигурации supervisor
RUN mkdir -p /etc/supervisor/conf.d

# Создание директорий для XFCE
RUN mkdir -p /root/.config/xfce4/xfconf/xfce-perchannel-xml && \
    mkdir -p /root/.config/xfce4/panel && \
    mkdir -p /root/.config/xfce4/xfconf

# Копирование конфигурации supervisor
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Создание скрипта для запуска
RUN echo '#!/bin/bash\n\
# Запуск supervisor\n\
/usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf\n\
# Ожидание\n\
tail -f /dev/null' > /start.sh && chmod +x /start.sh

# Создание рабочей директории
WORKDIR /workspace

# Инициализация ROS workspace
RUN mkdir -p /workspace/catkin_ws/src && \
    cd /workspace/catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Добавление в PATH
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Открытие портов
# 6080 - noVNC веб-интерфейс
# 5900 - VNC порт
# 11311 - ROS master
EXPOSE 6080 5900 11311

# Команда по умолчанию
CMD ["/start.sh"]


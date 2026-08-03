在开发小车或嵌入式设备时，每次开机都手动输入 `devmem` 来配置寄存器确实非常繁琐。

之所以需要使用 `sudo` 和 `busybox`，是因为引脚复用（Pinmux）配置保存在物理寄存器中，在没有通过设备树（Device Tree）永久保存的情况下，系统重启后这些寄存器就会重置。

为了解决这个问题，可以通过**开机自动运行脚本（Systemd 服务）**，将上述命令变成开机自动配置。这样配置好之后，**每次开机您都不需要手动输入任何 busybox 命令，直接运行 Python 脚本即可。**

以下是实现开机自动配置的步骤：

---

### 第一步：创建配置脚本

在系统中新建一个自动配置引脚的 Shell 脚本，存放在 `/usr/local/bin`：

1. **新建并编辑脚本文件**：
   ```bash
   sudo nano /usr/local/bin/enable_jetson_pwm.sh
   ```

2. **将以下配置内容复制进去**（这里同时为您写入了 Pin 32 和 Pin 33 的使能配置，方便您以后根据需要切换引脚）：
   ```bash
   #!/bin/bash
   # 使能物理引脚 32 (GPIO07 / PWM7)
   busybox devmem 0x02434080 32 0x404

   # 使能物理引脚 33 (GPIO13 / PWM5)
   busybox devmem 0x02434040 32 0x401
   ```

3. **保存并退出**（在 nano 文本编辑器中，按 `Ctrl + O` 保存，然后按 `Enter` 确认，再按 `Ctrl + X` 退出）。

4. **赋予脚本执行权限**：
   ```bash
   sudo chmod +x /usr/local/bin/enable_jetson_pwm.sh
   ```

---

### 第二步：创建守护服务（Systemd Service）

创建一个系统级别的启动服务，让其在开机时自动以 `root` 权限调用上述脚本。

1. **新建并编辑服务文件**：
   ```bash
   sudo nano /etc/systemd/system/enable-pwm.service
   ```

2. **写入以下配置内容**：
   ```ini
   [Unit]
   Description=Enable Hardware PWM on Jetson Orin Nano Startup
   After=multi-user.target

   [Service]
   Type=oneshot
   ExecStart=/usr/local/bin/enable_jetson_pwm.sh
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
   ```

3. **保存并退出**（`Ctrl + O` -> `Enter` -> `Ctrl + X`）。

---

### 第三步：启用并运行此服务

1. **重新加载系统服务管理器**：
   ```bash
   sudo systemctl daemon-reload
   ```

2. **设置该服务为开机自启**：
   ```bash
   sudo systemctl enable enable-pwm.service
   ```

3. **现在手动启动该服务，立即生效（不需要重启即可测试）**：
   ```bash
   sudo systemctl start enable-pwm.service
   ```

您可以通过以下命令查看该服务的运行状态，如果显示绿色的 `active (exited)`，说明已经成功运行过，并且引脚配置已在后台完成了：
```bash
sudo systemctl status enable-pwm.service
```

---

### 配置完成后的日常使用流程：

1. **现在已经不需要再执行任何 `busybox devmem` 的命令了**，每次只要开机，系统就会自动在后台将 Pin 32 和 Pin 33 切换成 PWM 模式。
2. **免 `sudo` 运行 Python 脚本**：由于系统启动时已经将引脚配置为 PWM 模式，您运行 Python 脚本时，可以直接不带 `sudo`。虽然不带 `sudo` 会弹出一个 `/dev/mem` 警告（提示它无法进行 pinmux 校验），但因为系统已经提前配好了引脚，这个警告可以被**直接忽略**，PWM 仍然可以正常输出和控制。
   ```bash
   /home/jetson/miniconda3/envs/yolo_env/bin/python "/home/jetson/Project/new_littlecar2/littlecar2/jetson/scripts/jetson上测试/补光灯测试.py"
   ```
首次绑定仍先运行向导：

```bash
cd /home/cly/Jbgs
source env/env.sh
ros2 run galaxy_camera_dual camera_setup_wizard
```

现在新机直接运行即可：

```
cd /home/cly/Jbgs
./run.sh sim:=false
```

网卡名不需要手抄：向导会自动列出本机网卡并选择（只有一个有线网卡时直接回车采用）。
网卡名由系统按网卡 MAC 自动生成（如 `enx00e04c1e2b40` = `en`+`x`+MAC `00:e0:4c:1e:2b:40`），
**每台电脑都不同**。



请在新打开的终端中完成：

1. 接受网络自动修复；
2. 输入网卡名：`enx00e04c1e2b40`；
3. 允许删除陈旧首地址并完成 sudo 授权；
4. 在两次大恒预览中按 Enter 后选择 L/R；
5. 选择并确认红外画面、确认串口传感器数据。

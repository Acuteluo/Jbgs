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

输入网卡名：

```bash
enx00e04c1e2b40
```



请在新打开的终端中完成：

1. 接受网络自动修复；
2. 输入网卡名：`enx00e04c1e2b40`；
3. 允许删除陈旧首地址并完成 sudo 授权；
4. 在两次大恒预览中按 Enter 后选择 L/R；
5. 选择并确认红外画面、确认串口传感器数据。

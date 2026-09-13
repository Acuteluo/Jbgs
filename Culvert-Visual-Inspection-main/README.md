# 涵洞内部视觉检测

红外白热灰度流上的轻量级渗水检测：UVC + OpenCV 局部差分，不依赖厂家 SDK，也不需要训练标注。

方案见 [docs/红外传统渗水检测方案.md](docs/红外传统渗水检测方案.md)。

## 编译与运行

```bash
cd ~/视觉巡检机器人系统/demo
cmake -S . -B build
cmake --build build -j
./build/ir_seepage_demo --camera 2
```

已在 `demo/build` 时直接运行 `./ir_seepage_demo --camera 2`。窗口为背景场 \(B\) 上的温差轮廓，按 `q` 退出。

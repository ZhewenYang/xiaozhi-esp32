硬件基于基于ESP32S3CAM开发板，代码基于bread-compact-wifi-lcd修改
使用的摄像头是OV2640
注意因为摄像头占用IO较多，所以占用了ESP32S3的USB 19 20两个引脚
连线方式参考config.h文件中对引脚的定义

 
# 编译配置命令

**配置编译目标为 ESP32S3：**

```bash
idf.py set-target esp32s3
```

**打开 menuconfig：**

```bash
idf.py menuconfig
```

**选择板子：**

```bash
Xiaozhi Assistant -> Board Type ->面包板新版接线（WiFi）+ LCD + Camera
```

**编译烧入：**

```bash
idf.py build flash
```

## v15 修复：连拍与语音播报分时互斥（2026-11-18）

- 相机 worker 在开启 20MHz XCLK 前，先等待 speaking/notifying 结束且播放队列排空；
- 等待只阻塞相机 worker，不阻塞主线程；超过 45 秒返回明确错误；
- AudioService 在 speaking/notifying 期间禁止空闲超时关功放，避免网络抖动导致说到一半静音；
- XCLK 仍仅在抓帧期间开窗，避免 GPIO15 20MHz 干扰 Wi-Fi 造成 UDP Opus 丢包（esp32-camera #869）。

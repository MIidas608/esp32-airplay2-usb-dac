# ESP32-S3 AirPlay 2 接收器（USB DAC 版）

基于 [rbouteiller/airplay-esp32](https://github.com/rbouteiller/airplay-esp32) 修改，音频输出由 I2S 改为 **USB DAC（USB Audio Class Host）**。ESP32-S3 通过 USB 驱动外置声卡，将 iPhone 的 AirPlay 音频输出到耳机或功放。

## 主要改动

- **USB DAC 输出**：新增 `uac_output` 组件和 `audio_output_usb_host` 后端，通过 USB Host 驱动外置声卡替代 I2S。支持 UAC 1.0 设备，CX21988 已测试可用。
- **44.1kHz 直通**：AirPlay 音频采样率为 44.1kHz，直接输出不做重采样。原版默认转换为 48kHz，需要额外一次 sinc 重采样。
- **5 段 EQ 与响度均衡**：可在网页中调节，低音量时自动增强低频与高频。
- **音量记忆**：重启后恢复上次音量。
- **中文配置页**：单页界面，支持深色与浅色切换。
- 此外包含若干稳定性修复。

## 硬件要求

- ESP32-S3 开发板，16MB Flash + 8MB PSRAM（PSRAM 为 AirPlay 2 必需）
- UAC 1.0 协议的 USB 声卡或转接头
- 板载 USB 口到声卡的连接线或转接头

关于 VBUS 供电：ESP32-S3 的 USB 口默认为 device 模式，用作 host 驱动外置声卡时需要板级提供 5V。以 YD-ESP32-S3 为例，需短接板背面的 USB-OTG 焊盘，否则声卡无法上电、不能枚举。不同板子的 VBUS 供电路径不同，使用前需确认。

## 构建

需要 ESP-IDF 5.5.3 或更高版本（开发时用的是 5.5.5）。

```bash
git clone https://github.com/MIidas608/esp32-airplay2-usb-dac.git
cd esp32-airplay2-usb-dac
idf.py set-target esp32s3
idf.py build
```

构建时会从组件仓库自动下载 mdns、esp_audio_codec、libsodium、led_strip 等依赖。`usb_host_uac`、`usb`、`cmake_utilities` 已经放在 `components/` 下，不走自动下载。

## 烧录

```bash
idf.py -p COMx flash
```

一条命令会烧完 bootloader、分区表、程序（app）和网页（storage/SPIFFS）。网页能被一起烧是因为 `CMakeLists.txt` 里给 SPIFFS 镜像加了 `FLASH_IN_PROJECT`。

要单独更新某个分区（例如只换网页）时，用 esptool 按偏移烧：

| 内容 | 偏移 | 产物 |
|---|---|---|
| 程序 | `0x20000` | `build/airplay2-receiver.bin` |
| 网页 | `0x620000` | `build/storage.bin` |

分区表用 `components/boards/partitions.csv`（8MB 版）：两个 3MB 的 app 分区（OTA 双区）+ 一个 1.875MB 的 storage 分区。

首次启动会进入配置热点（SSID 形如 ESP32-AirPlay-Setup）。连接后访问 `http://192.168.4.1` 配置 WiFi，完成后设备会出现在 iPhone 的 AirPlay 列表中。

## 网页配置

配置好 WiFi 后，通过浏览器访问设备 IP（或 `设备名.local`）。配置页为单页结构，包含以下标签页：

- 状态：连接信息、DAC 状态、播放状态、网络测速
- 网络：WiFi 扫描与连接
- 均衡：5 段 EQ 滑块与响度开关
- 日志：实时串口日志
- 设置：设备名、OTA 升级、重启

OTA 升级仅覆盖程序分区（app）。修改网页（storage 分区）后需通过串口烧录。

## 致谢

- [rbouteiller/airplay-esp32](https://github.com/rbouteiller/airplay-esp32) —— 本项目的基础，AirPlay 2 协议实现
- [Espressif](https://github.com/espressif) —— ESP-IDF、usb_host_uac、esp_audio_codec
- [Shairport Sync](https://github.com/mikebrady/shairport-sync) 与 [openairplay/airplay2-receiver](https://github.com/openairplay/airplay2-receiver) —— AirPlay 协议的参考实现（上游项目的参考来源）

## 许可证

本仓库继承上游 [rbouteiller/airplay-esp32](https://github.com/rbouteiller/airplay-esp32) 的非商业许可，仅限非商业用途。商用需要联系上游作者。详见 [LICENSE](LICENSE)。

## 免责声明

本项目基于协议分析，与 Apple Inc. 无关。不保证兼容未来的 iOS / macOS 版本，按原样提供，不附带任何担保。具体见 [DISCLAIMER.md](DISCLAIMER.md)。

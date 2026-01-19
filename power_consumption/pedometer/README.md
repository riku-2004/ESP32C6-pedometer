# ペドメーター消費電力測定

## 概要
PPK2を用いてrm, rl, cm, cl の4構成でペドメーターの消費電力を測定する。

## 構成
| 略称 | 説明 | プロジェクトパス |
|------|------|----------------|
| **cm** | C / Main Core | `cm/pedometer/` |
| **cl** | C / LP Core | `cl/pedometer/` |
| **rm** | mruby/c / Main Core | `rm/` (pedometer.c を include) |
| **rl** | mruby/c / LP Core | `rl/` (pedometer.c を include) |

## 環境
- 評価ボード: ESP32-C6-DevKitC-1 N8, v1.3
- 電流計: Nordic Power Profiler Kit 2 (PPK2)
- センサー: Analog Devices ADXL367 (加速度センサー)

## PPK2 設定
- モード: Source Meter Mode
- 供給電圧: 3.3V
- サンプリング: 10^5 samples/sec

## 接続
```
PPK2 VOUT ──→ ESP32-C6 3V3
PPK2 GND  ──→ ESP32-C6 GND
```

## ビルド・フラッシュ手順

### cm (C / Main Core)
```bash
cd cm/pedometer
idf.py build flash
```

### cl (C / LP Core)
```bash
cd cl/pedometer
idf.py build flash
```

### rm (mruby/c / Main Core)
```bash
cd rm
# main/main.c で #include "pedometer.c" を確認
idf.py build flash
```

### rl (mruby/c / LP Core)
```bash
cd rl
# main/main.c で #include "pedometer.c" を確認
idf.py build flash
```

## 測定手順
1. PPK2の電源出力を無効化
2. ESP32-C6にプログラムをフラッシュ
3. PPK2で測定を開始
4. PPK2の電源出力を有効化
5. 100歩カウント × 10〜20セット実施
6. 測定を停止し、データを保存
7. 次の構成に切り替えて繰り返し

## 歩数計測の確認
各構成でシリアルモニタを確認し、歩数がカウントされていることを確認：
```bash
idf.py monitor
```

## データ分析
- PPK2からCSV出力
- 各セットの消費電力 [mJ] と時間 [s] を記録
- 平均消費電力を計算・比較

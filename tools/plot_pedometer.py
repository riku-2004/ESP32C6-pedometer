#!/usr/bin/env python3
"""
歩数計データのリアルタイムグラフ表示

使い方:
  python plot_pedometer.py COM3
  
グラフ表示:
  - 青線: 生の加速度マグニチュード
  - 緑線: フィルタ後の値
  - 赤点線: 閾値
  - オレンジ縦線: 歩数検出ポイント

Ctrl+C で終了
"""

import sys
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

# 日本語フォント設定
plt.rcParams['font.family'] = ['MS Gothic', 'Yu Gothic', 'Meiryo', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# 設定
MAX_POINTS = 200  # グラフに表示するデータ点数
BAUD_RATE = 115200

# データ格納用
times = deque(maxlen=MAX_POINTS)
magnitudes = deque(maxlen=MAX_POINTS)
filtered_vals = deque(maxlen=MAX_POINTS)
thresholds = deque(maxlen=MAX_POINTS)
step_times = []  # 歩数検出時刻
last_step_count = 0

# シリアルポート
ser = None

def parse_line(line):
    """DATA,時間,マグニチュード,フィルタ値,閾値,歩数 形式をパース"""
    try:
        if line.startswith("DATA,"):
            parts = line.strip().split(",")
            if len(parts) == 6:
                return {
                    'time': int(parts[1]),
                    'mag': int(parts[2]),
                    'filtered': int(parts[3]),
                    'threshold': int(parts[4]),
                    'steps': int(parts[5])
                }
    except:
        pass
    return None

def update(frame):
    """グラフ更新"""
    global last_step_count
    
    # シリアルから読み取り
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore')
            data = parse_line(line)
            if data:
                times.append(data['time'] / 1000.0)  # 秒に変換
                magnitudes.append(data['mag'])
                filtered_vals.append(data['filtered'])
                thresholds.append(data['threshold'])
                
                # 新しい歩数を検出
                if data['steps'] > last_step_count:
                    step_times.append(data['time'] / 1000.0)
                    last_step_count = data['steps']
        except:
            pass
    
    # グラフをクリア
    ax.clear()
    
    if len(times) > 0:
        # データプロット
        ax.plot(list(times), list(magnitudes), 'b-', label='生マグニチュード', alpha=0.7)
        ax.plot(list(times), list(filtered_vals), 'g-', linewidth=2, label='フィルタ後')
        ax.plot(list(times), list(thresholds), 'r--', label='閾値')
        
        # 歩数検出ポイントを縦線で表示
        for st in step_times:
            if len(times) > 0 and st >= min(times):
                ax.axvline(x=st, color='orange', linestyle='-', alpha=0.7, linewidth=2)
        
        ax.set_xlabel('時間 (秒)')
        ax.set_ylabel('加速度 (LSB)')
        ax.set_title(f'歩数計モニター - 歩数: {last_step_count}')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Y軸の範囲を調整
        if len(magnitudes) > 0:
            y_min = min(min(magnitudes), min(filtered_vals), min(thresholds)) - 500
            y_max = max(max(magnitudes), max(filtered_vals), max(thresholds)) + 500
            ax.set_ylim(y_min, y_max)
    
    return []

def main():
    global ser, fig, ax
    
    if len(sys.argv) < 2:
        print("使い方: python plot_pedometer.py <COMポート>")
        print("例: python plot_pedometer.py COM3")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print(f"ポート {port} に接続中...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        print(f"接続成功！ボーレート: {BAUD_RATE}")
    except Exception as e:
        print(f"接続エラー: {e}")
        sys.exit(1)
    
    # グラフ設定
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.canvas.manager.set_window_title('歩数計リアルタイムモニター')
    
    # アニメーション開始
    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    
    print("グラフ表示中... Ctrl+C または ウィンドウを閉じて終了")
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            ser.close()
        print("終了")

if __name__ == "__main__":
    main()

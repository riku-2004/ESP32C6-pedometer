# mruby Coprocessor (`rl`) の有用性評価ガイド

## 評価の目的
mruby coprocessor（`rl`: mruby/c on LP Core）の有用性を示すための指標と比較方法

## 評価指標と比較方法

### 1. 消費電力（Power Consumption） - 最重要指標

#### 比較対象
- **`rm` vs `rl`**: Ruby版でMain CoreとLP Coreの消費電力比較
  - **目的**: LP Coreを使うことでRubyプログラムの消費電力をどれだけ削減できるか
  - **フォルダ**: `power_consumption/pedometer/`
  - **測定方法**: PPK2で100歩カウント × 10〜20セットの消費電力[mJ]を測定

- **`cl` vs `rl`**: LP CoreでC言語とRubyの消費電力比較
  - **目的**: LP Core上でRubyを使ってもC言語と同等の低消費電力を実現できるか
  - **フォルダ**: `power_consumption/pedometer/`
  - **期待**: `rl`は`cl`より若干高いが、`rm`より大幅に低い

- **`cm` vs `rl`**: Main CoreのC言語とLP CoreのRubyの比較
  - **目的**: 開発効率（Ruby）と低消費電力（LP Core）の両立を示す
  - **フォルダ**: `power_consumption/pedometer/`

#### 測定手順
1. `cm/pedometer/` でビルド・フラッシュ・測定
2. `cl/pedometer/` でビルド・フラッシュ・測定
3. `rm/` で `pedometer.c` をincludeしてビルド・フラッシュ・測定
4. `rl/` で `pedometer.c` をincludeしてビルド・フラッシュ・測定
5. PPK2のCSVデータから平均消費電力を計算・比較

#### 期待される結果
```
cm (C/Main) > rm (Ruby/Main) > rl (Ruby/LP) ≈ cl (C/LP)
```
- `rl`は`rm`より大幅に低消費電力（LP Coreの効果）
- `rl`は`cl`と同等またはやや高い（Rubyのオーバーヘッドは許容範囲）

---

### 2. コードサイズ（Code Size）

#### LP Core上のコードサイズ比較
- **`cl` vs `rl`**: LP Core上でC言語とRubyのコードサイズ比較
  - **フォルダ**: `code_size/`
  - **`cl`**: `cl-${APPNAME}.ulp_core_main.map` から関数サイズを合計
  - **`rl`**: `rl-${APPNAME}.txt` から `PROF allocated = ` (Metadata) と `GC allocated = ` (Generated Code) を確認

#### Main Core上のランタイムサイズ比較
- **`rm` vs `rl`**: Main Core上のmruby/cランタイムサイズ比較
  - **フォルダ**: `runtime_code_size/`
  - **測定方法**: `idf.py size-components` を実行
  - **ファイル**: `mrubyc-size-components.txt` (rm) と `mrubycopro-size-components.txt` (rl)
  - **確認項目**: `libmruby.a` のサイズ

#### 期待される結果
- LP Core上のコードサイズ: `rl`は`cl`より大きい（メタデータと生成コードのオーバーヘッド）
- Main Core上のランタイム: `rl`は`rm`より大きい可能性（coprocessor用の追加コード）

---

### 3. 開発効率・保守性（Development Efficiency）

#### 比較方法
- **コードの可読性**: `rl/main/pedometer.rb` vs `cl/pedometer/main/main.c`
- **コード行数（LOC）**: Ruby版とC言語版の行数比較
- **実装の容易さ**: アルゴリズム変更時の修正箇所と難易度

#### 評価ポイント
- Rubyの高級言語機能（クラス、メソッド、配列操作など）の活用
- アルゴリズムの変更が容易か
- デバッグのしやすさ

---

### 4. 実行性能（Performance）

#### 比較対象
- **`rm` vs `rl`**: Ruby版でMain CoreとLP Coreの実行速度比較
- **`cl` vs `rl`**: LP Core上でC言語とRubyの実行速度比較

#### 測定方法
- 100歩カウントに要する時間を測定
- シリアルモニタでタイムスタンプを確認

#### 期待される結果
- `rl`は`rm`より遅い可能性（LP Coreのクロック周波数が低い）
- `rl`は`cl`より遅い可能性（Rubyのインタープリターオーバーヘッド）
- ただし、歩数計測の精度は同等である必要がある

---

## 推奨される評価順序

### Phase 1: 基本比較（必須）
1. **消費電力**: `rm` vs `rl` vs `cl` vs `cm`
   - フォルダ: `power_consumption/pedometer/`
   - これが最も重要な指標

### Phase 2: コードサイズ（重要）
2. **LP Coreコードサイズ**: `cl` vs `rl`
   - フォルダ: `code_size/`
3. **Main Coreランタイムサイズ**: `rm` vs `rl`
   - フォルダ: `runtime_code_size/`

### Phase 3: 補足評価（オプション）
4. **開発効率**: コードの可読性・保守性の比較
5. **実行性能**: 実行速度の比較

---

## 評価結果のまとめ方

### 表形式での比較
| 指標 | cm | cl | rm | rl | 評価 |
|------|----|----|----|----|------|
| 消費電力 [mJ/100歩] | - | - | - | - | rl < rm を示す |
| LP Coreコードサイズ [bytes] | - | - | - | - | rl > cl だが許容範囲 |
| Main Coreランタイム [bytes] | - | - | - | - | rl > rm だが許容範囲 |
| コード行数 [LOC] | - | - | - | - | rl < cl を示す |

### 結論として示すべきポイント
1. **低消費電力**: `rl`は`rm`より大幅に低消費電力（LP Coreの効果）
2. **開発効率**: Rubyで書けるため、C言語より開発・保守が容易
3. **実用性**: `rl`は`cl`と同等の低消費電力を維持しつつ、Rubyの利点を享受

---

## 測定時の注意点

1. **測定条件の統一**: 同じセンサー、同じ環境、同じ歩数で測定
2. **複数回測定**: 統計的有意性のため10〜20セット測定
3. **エラーハンドリング**: `rl`でI2Cエラーが発生している場合は修正が必要
4. **精度の確認**: 各構成で歩数計測の精度が同等であることを確認

---

## 参考資料
- `power_consumption/pedometer/README.md`: 消費電力測定手順
- `code_size/README.md`: コードサイズ測定手順
- `runtime_code_size/README.md`: ランタイムサイズ測定手順

# 邊緣運算

1. **Python Tensorflow** MLP(算分類), LSTM(時間), CNN(圖片)
2. **Save weight.h5** 權重和模型檔案
3. **float64 -> int8**
4. **save TFLite .tf** 邊緣運算檔案
5. **轉檔案 .tf -> .h** (把 .tf 檔案內容 bytes 的內容 轉成 c 語言的 array)

---
**以上是 PC Python, ↓**

**以下的部分 GCC+ ARM (M3 以上) M3, M33, M4.... TFLite 的原始程式碼**  

6. **model.h**
7.predict model,  tflite api predict()
8. 輸出答案


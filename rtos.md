## freeRTOS核心組成
### task
- status
    - Running：目前正在 CPU 上執行
    - Ready：可執行但尚未輪到
    - Blocked：等待事件（Queue、Semaphore、Delay）
    - Suspended：被暫停，不會被排程
    - Dormant：尚未啟動或已被刪除
- 常用 API

### Scheduler 排程器

### Queue 佇列
-  FIFO（First-In First-Out）常用於感測器資料傳遞、命令分派等模組化架構。
-  阻塞與超時
   -  阻塞：任務會等待資料或空間
   -  非阻塞：立即返回失敗
   -  指定超時：在限制時間內等待資料或空間
-  
### 信號量（Semaphore）與互斥鎖（Mutex）


###  事件群組（Event Groups）
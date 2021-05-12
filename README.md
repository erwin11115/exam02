# hw3

當編譯 main.cpp 5 之後。

開啟screen (sudo screen /dev/ACM0)
按下 reset 確保開始執行的準確。
screen 會顯示 『連接的過程』是否正常。
直到顯示 waiting ( 代表程式等待RPC指令到來)

執行demo.py (會自動傳輸RPC指令)
進入 gesture UI mode ，搖晃mbed版(三角形、圓形)，來調整角度參數(30，35，40，45，50)
按下USER鍵確認
python會讀取publish的訊息，並傳輸指令停止gesture UI function，開啟 tilt angle detection mode。

在led燈閃爍的時候要mbed平放於桌面紀錄參考值
閃爍過後便會開始顯示現在的角度

當現在角度大於角度參數的時候，tilt angle detection function 便會停下。
等待下次RPC指令。

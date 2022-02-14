# Picking Simulation
検証済み動作環境 :
* Ubuntu 18.04
* python 3.7.7
* pip 21.3.1

依存関係 :  
1. gqcnn (https://github.com/BerkeleyAutomation/gqcnn)
```
Instlation steps :
git clone https://github.com/BerkeleyAutomation/gqcnn.git
cd gqcnn
pip install .  #pip 利用時 (ros, docker はドキュメント参照)
```  
2. pybullet (https://github.com/csingh27/Bin-Picking-Simulation-using-Pybullet 参考)
```
Instlation steps :
pip3 install --user gym  
pip3 install pybullet --upgrade --user
pip install attrdict
```  

実行 :
```
git clone https://github.com/sice-manip-young/PickingSimulation.git
cd PickingSimulation
python run.py
``` 

バグ :
* キャプチャした深度画像のスケール
* (おそらく)↑が原因で推論時に NoValidGraspsException で強制終了

ToDo :
* 画像ピクセルスケールからメートルスケールへの自動変換
* 訓練データサンプル用のランダムポリシー
* 把持成否判定

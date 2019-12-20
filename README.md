# collect_calib_data

Velodyneをpitch方向に動かすための雲台

## 自動制御雲台の使用

- ±1 degのバックラッシュが機械的に生じる.
- が, ポテンショメータがバックラッシュも正確に取得するため,**特に気にしないのであれば**ポテンショメータの値を基にVelodyne点群を座標変換すれば問題はない．
- 1 deg単位で角度変換が可能.

## 必要パッケージ

Velodyneがある必要がある.  
また, Velodyneを運用するパッケージとrosserialを別途準備必須.

```
$ sudo apt-get install ros-<version>-rosserial
$ sudo apt-get install ros-kinetic-velodyne
<~/catkin_ws/src>$ git clone https://github.com/ros-drivers/velodyne.git
```

## 以降気分次第で追記

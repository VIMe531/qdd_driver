# qdd_driver
複数種類のQDDモータを動かすためのコード群．

SteadyWin製GIM8108-8とXiaomi製Cybergearに対応済．
ROBSTRIDEやmyActuatorなど他のモータにも今後対応させたい（願望）

## GIM8108-8
"GIM81088"ディレクトリ内の"GIM81088_Driver.h"をインクルードして使用する．"test"ディレクトリ内にサンプルコード（GIM81088_test.cpp）有．C++11でのビルドを想定．以下のようにしてビルドする．

```
make -f GIM81088_test.mk

// clear
make -f GIM81088_test.mk clean
```

## Cybergear
"Cybergear"ディレクトリ内の"Cybergear_Driver.h"をインクルードして使用する．"test"ディレクトリ内にサンプルコード（Cybergear_test.cpp）有．C++11でのビルドを想定．以下のようにしてビルドする．

```
make -f Cybergear_test.mk

// clear
make -f Cybergear_test.mk clean
```

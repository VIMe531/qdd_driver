# qdd_driver
複数種類のQDDモータを動かすためのコード群．

Xiaomi製CybergearとROBSTRIDE Dynamics製ROBSTRIDE00、SteadyWin製GIM8108-8に対応済．
myActuatorなど他のモータにも今後対応させたい

## Cybergear
"Cybergear"ディレクトリ内の"Cybergear_Driver.h"をインクルードして使用する．"test"ディレクトリ内にサンプルコード（Cybergear_test.cpp）有．Gnu make & C++11でのビルドを想定．以下のようにしてビルドする．

```
make -f Cybergear_test.mk

// clear
make -f Cybergear_test.mk clean
```

## ROBSTRIDE
"ROBSTRIDE"ディレクトリ内の"RS00_Driver.h"をインクルードして使用する．"test"ディレクトリ内にサンプルコード（RS00_test.cpp）有．Gnu make & C++11でのビルドを想定．以下のようにしてビルドする．

```
make -f RS00_test.mk

// clear
make -f RS00_test.mk clean
```

## GIM8108-8
"GIM81088"ディレクトリ内の"GIM81088_Driver.h"をインクルードして使用する．"test"ディレクトリ内にサンプルコード（GIM81088_test.cpp）有．Gnu make & C++11でのビルドを想定．以下のようにしてビルドする．

```
make -f GIM81088_test.mk

// clear
make -f GIM81088_test.mk clean
```

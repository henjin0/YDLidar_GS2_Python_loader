# YDLidar_GS2_Python_loader

# 使い方
venvによる仮想環境で下記の実行環境を構築し、`python main.py`を実行してください。
また、matplotlibにてグラフ化する際にはmain.pyの`note: グラフ化して確認したい際にコメントアウト解除する。`とメモされているコメントアウトをすべて解除してコードが通るようにしてください。

# 実行環境

下記環境で動作確認ができています。

```
python: Python 3.9.1
pip: pip 22.1.2
```

`pip install -r requirement.txt`でパッケージをインストールすれば一通り実行環境が組めます。

# Raspberry piで動かしたい場合は
一応下記環境にて動作確認済みです。

```
RaspberryPi 4B 8GB model
[OS]
Name: Raspberry Pi OS with desktop
Release date: April 4th 2022
System: 64-bit
Kernel version: 5.15
Debian version: 11 (bullseye)
```

先にUARTの設定を実施する必要があるため、下記リンクを参考にUART0を使えるようにする。そして、main.pyの接続先のserialを`/dev/serial0`にして`python main.py`を実行すると確認可能。

https://www.ingenious.jp/articles/howto/raspberry-pi-howto/gpio-uart/

# License
Please see [LICENSE](LICENSE).

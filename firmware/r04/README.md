# r04

作者が設計中のc07を最適制御するために開発中の制御方式です。

RP2040向けにCとRaspberry Pi Pico SDKで実装しています。

制御の流れは **IncCond → パネル電圧目標 → 電圧PID → L1電流目標 → 電流PI → Duty** です。

[制御本体](src/core/c07_control.c) ／ [実行・割り込み処理](src/main.c) ／ [起動・停止処理](src/startup.c) ／ [入出力設定](src/board_config.h)

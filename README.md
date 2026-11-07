# 太陽光充電システム
この制作は太陽光発電システムの設計のポートフォリオです。
シミュレーションでのみ動作の確認を行っており実機の作成まで行っていません。

400ｗのパネルを二枚直列に繋いだ800ｗの充電システムです。
バッテリーには51.2Vのリン酸鉄リチウムイオンバッテリーを採用しています。
HIGH側、LOW側にNchMOSFetを設置した同期整流、BUCK方式です。
パネル側、バッテリー側両方にパッシブで動作する保護装置をつけています。


## 掲載資料

- [部品表（Excel）](docs/bom/c07_BOM.xlsx)
- [回路図（PDF・3ページ）](docs/schematic/C07_Solar.pdf)
- [KiCad回路図データ](hardware/c07/project/)
- [KiCad PCBデータ](hardware/c07/pcb-wip/)
- [制御コード（C）](firmware/r04/)
- [MATLAB回路図データ](hardware/c07/matlab/)

ArmISIK
======================
ArmISIKは３軸マニピュレータの逆運動学を得コンポーネントです    
軸配置は決められていますが、軸間距離などは自由に設定できます  
自作のロボットアームの制御などに使用できます  

本RTCを用いたデモンストレーションをYoutubeでご覧になれます   
[アームロボットによるカラーボールのお片づけ][video]  
[video]: http://www.youtube.com/watch?v=Mtw2xgm07t8 
    
動作確認環境
------
Python:  
2.6.6  

OS:  
Windows 7 64bit / 32bit  
Ubuntu 10.04 LTS / 12.04 LTS 32bit  

対応RTC:  
[RsServoManager][servo]  
[ArmISColorBall][color]  

[servo]: https://github.com/HiroakiMatsuda/RsServoManager
[color]: https://github.com/HiroakiMatsuda/ArmISColorBall

ファイル構成
------
ArmISIK 
│―ArmISIK.py  
│―calcik.py  
│―ini   
│　　│―ikconfig.ini    
│  
│―rtc.conf  

* ArmISIK.py  
RTC本体です  
* calcik.py  
逆運動学の計算を行なっています
* ikconfig.ini  
ロボットアームのボディパラメータなどを設定します  
* rtc.conf  
ポートの設定や動作周期を設定できます  

注:本RTCにおいてユーザーが操作すると想定しているファイルのみ説明しています  

RTCの設定はiniファイルを通して行えるので、簡単に設定を変えられます  
iniファイルはActivate時に読み込むので、設定を変更した場合はDeactiveにしたあとActivateしてください

RTCの構成
------  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readme_01.png" width="500px" />    
[ArmISColorBall][color], [RsServoManager][servo]などの対応するRTCを接続することでアームロボットを制御できます  

* motion port :OutPort  
データ型; TimedLongSeq  
[Flag, Id, Position, Time]  
 ・  `Flag` :  モーションの同期用フラグ  
0:同期なし  
移動指令は全て同期なしで指令  
 ・  `Id` :  サーボID  
指令を送るサーボモータのIDを指定します      
 ・  `Position` :  移動位置  
サーボモータへ角度 [0.1 deg]を指定します     
 ・  `Time` :  移動時間  
指定位置までの移動時間 0~16383 [msec]の間で指定します  

* on_off port :OutPort  
データ型; TimedLongSeq  
[Flag, Id, Torque mode]  
 ・  `Flag` :  モーションの同期用フラグ  
0:同期なし  
1:同期あり  
Flagを1にすると設定したサーボモータ全てに同時に指令を送ります  
個別のトルク保持指令ボタンを押した場合は同期なしモード  
全体のトルク保持指令ボタンを押した場合は同期モード   
 ・  `Id` :  サーボID  
指令を送るサーボモータのIDを指定します    
 ・  `Toruque mode` :  トルクモード指定  
0:トルクをオフにします（ボタンが赤色のとき）  
1:トルクをオンにします（ボタンが緑色のとき）  
 
* command port :InPort   
 データ型; TimedLongSeq  
 データ長は最大5で、コマンドの種別によってはcommand以降のデータは無視されます  
常にデータ長5で通信する場合は、コマンド以降はどのように設定しても構いません  
 ・  Command = 0 :   [0]  
イニシャルポジションに移動します  
  
 ・  'Command = 1 :   [1, Pos_x, Pos_y, Pos_z, Gripper]  
指定されたXYZ座標に移動します  

 ・  `'Pos_x` : ロボット座標系のX座標を指定します   
 ・  `'Pos_y` : ロボット座標系のY座標を指定します   
 ・  `'Pos_z` : ロボット座標系のZ座標を指定します   
 ・  `'Gripper` : ロボットのグリッパ開閉量を指定します   
このときに指定座標がロボットアームの範囲外であるときは、ロボットアームの関節値に0を発行します  
また設定した角度リミットに達した場合は、各関節の指令値はリミット値に設定されます  

 ・  Command = 2 :   [2]  
グリッパを閉じ、イニシャルポジションまで移動します  
把持対象物の付近までCommand = 1で接近し、Command = 2で初期値まで持ち帰るとを想定しています  

 ・  Command = 1000 :   [1000]  
全てのサーボモータへTorque ON司令を発行します  

 ・  Command = 1001 :   [1001]  
全てのサーボモータへTorque OFF司令を発行します  
その他のコマンドについては今後のバージョンで追加していきます  
 
* sensor port :InPort（現在のバージョンでは利用できません）  
データ型; TimedLongSeq  
[Id, Angle, Time, Speed, Load, Temperature, Voltage]  
 ・  `'Angle` :  現在位置 [0.1 deg]  
 ・  `'Time` :  現在時間 [0.1 sec]  
現在時間はサーボが指令を受信し、移動を開始してからの経過時間です   
 ・  `'Speed` :  現在スピード [deg / sec]  
現在の回転スピードを取得できますが、この値は目安です。  
 ・  `'Load` :  現在負荷 [mA]  
サーボに供給されている電流を返しますが、この値は目安です。  
 ・  `Tempreture` :  現在温度 [degree celsius]  
この値はセンサの個体差により±3 [degree celsius] 程度の誤差があります  
 ・  `'Voltage` :  現在電圧 [10 mV]  
この値はセンサの個体差により±0.3 [V]程度の誤差があります   

使い方
------
###1. 使用するロボットアームを設定する###
ikconfig.iniをテキストエディタなどで開き編集します  
なお各リンクの番号は下図のようにアームに対応します  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readme_02.png" width="100px" /> 

BODY  
  ・ ``link_1 = X```  
第1リンクの長さ [mm]を指定します  
 
  ・ ``link_2 = X```  
第2リンクの長さ [mm]を指定します  

  ・ ``link_3 = X```  
第3リンクの長さ [mm]を指定します  
 
  ・ ``link_4 = X```  
第4リンクの長さ [mm]を指定します  

  ・ ``link_5 = X```  
第5リンクの長さ [mm]を指定します  

CALIBRATION  
  ・ ```calb_1 =  X```  
第1関節のアクチュエータに指令値0を送ったときに、アームが正面を向くようオフセット値を設定します　
単位は[0.1 deg]です  

  ・ ```calb_2 =  X```  
第2関節のアクチュエータに指令値0を送ったときに、アームが直立するようオフセット値を設定します　
単位は[0.1 deg]です     

  ・ ```calb_3 =  X```  
第3関節のアクチュエータに指令値0を送ったときに、アームが直立するようオフセット値を設定します　
単位は[0.1 deg]です  

  ・ ```calb_4 =  X```  
グリッパーに指令値0を送ったときに、アームが閉じるようオフセット値を設定します　
単位は[0.1 deg]です  


LIMIT  
 ・ ```servo1_min =  X```  
第1関節のアクチュエータが稼働できる最小値を設定します  
このリミッターを設けることで、ロボットアームの自己干渉をある程度回避できます  
関節指令値がこの値を下回る場合は、設定した最小値に指令値が変更されます  
単位は[0.1 deg]です  

  ・ ```servo1_max =  X```  
第1関節のアクチュエータが稼働できる最大値を設定します  
このリミッターを設けることで、ロボットアームの自己干渉をある程度回避できます  
関節指令値がこの値を上回る場合は、設定した最大値に指令値が変更されます  
単位は[0.1 deg]です  

以下同様に設定してください  
  ・ ```servo2_min =  X```  
  ・ ```servo2_max =  X```  
  ・ ```servo3_min =  X```  
  ・ ```servo3_max =  X```   
  ・ ```servo4_min =  X```   
  ・ ```servo4_max =  X```   

SERVO  
・ ```move_time =  X```  
サーボモータの移動時間を指定します  
単位は[10ms]です
  
###2. サンプルシステム：カラーボールのお片づけ###
この項目ではArmISIK, [ArmISColorBall][color], [RsServoManager][servo]を使用してカラーボールを片けるサンプルシステムを用いて本RTCの説明を行います    

なお、本システムで使用するロボットアーム'ArmIS type0'は双葉電子工業（株）のコマンド方式サーボモータである、RS405CBを４つ使用して制作された、全長280 [mm]の卓上小型ロボットアームです  
外観を以下に示します  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readme_03.jpg" width="300px" /> 

本システムの概要は以下のようになり、天井カメラ(USBカメラ)、ロボットアーム、トレー、カラーボールが必要です  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readme_04.png" width="400px" /> 

1. 環境を整える  
本システムの概要に従い、天井カメラ、ロボットアーム、トレー、カラーボールをセットします  

2. RsServoManger, ArmISIK, ArmISColorBallを起動する  
ロボットアームとPCの通信用シリアルケーブル、カメラを接続し、ロボットアームの電源を入れます    
その後各RTCを起動し、接続後にRsServoManger, ArmISIK, ArmISColorBallの順に起動します  
この起動手順は必ずしも守る必要はありませんが、下層のRTCより起動することで無用なトラブルを回避する狙いがあります  

3. ArmISColorBallのキャリブレーションを行う  
ArmISColorBallの下図のようにカメラ画面に表示されている、黄色い十字円の交点とArmIS type0台座のマーカーの位置を合わせます  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/raeadme_05.png" width="400px" />

 'u'ボタンを押すことで円の半径が1[pixel]ずつ広がり、'd'ボタンを押すことで円の半径が1[pixel]ずつ小さくなります  
十字円の位置と大きさを合わせることで、アームの台座の長さから1[pixel]がロボット座標系でx[mm]に相当するのかを測定します  
この簡易キャリブレーションシステムで、卓上ロボットアームを円滑に運用できます  
この時点でカラーボールやトレーが設置されていても構いませんが、邪魔な場合は一度除去し、キャリブレーション後に再度設置してください  

4. ロボットアームの初期化を行う  
ArmISColorBallのカメラ画面に入力のフォーカスを合わせ、'o'ボタンを押しTorque ON命令を発行します  
その後'i'ボタンを押し、イニシャルポジションにロボットアームを移動させます  
本システムではこのときのグリッパの下にトレーを設定します  

5. 認識しているカラーボールを拾う  
ArmISColorBallのカメラ画面には以下のように認識しているボールに、円が描かれ座標が保持されています  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readm_06.png" width="400px" />

 'r', 'g', 'b', 'p'のキーを押すと、それぞれ、赤、緑、青、桃のボールの付近までアームが移動します  
移動後に'c'ボタンを押すことで、アームがボールを把持しトレーまで運び格納します  
これを繰り返すことでロボットアームが卓上のカラーボールをお掃除してくれます  

注意：カラーボール検出アルゴリズムの都合で、1つの色のボールは1つまでしかカメラに映さないでください  
      
以上が本RTCの使い方のサンプルとなります  

ライセンス
----------
Copyright &copy; 2012 Hiroaki Matsuda  
Licensed under the [Apache License, Version 2.0][Apache]  
Distributed under the [MIT License][mit].  
Dual licensed under the [MIT license][MIT] and [GPL license][GPL].  
 
[Apache]: http://www.apache.org/licenses/LICENSE-2.0
[MIT]: http://www.opensource.org/licenses/mit-license.php
[GPL]: http://www.gnu.org/licenses/gpl.html
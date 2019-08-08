# Overview  
RoboCup2019に向けて開発したマニピュレータを制御するパッケージ  
物体把持、物体設置、モータ調整を行えます。  

# How to use  
モータを制御できるようにセットアップを行います。  

    $ roslaunch manipulation motor_setup.launch  
物体把持を行う場合は  

    $ rosrun manipulation manipulation.py  
を実行して下さい。(他パッケージは省略)  
アームの動作実験、モータの調整を行う場合は  

    $ rosrun manipulation echo_motor_state.sh  
	$ rosrun manipulation experiment_motor.py  
を実行して下さい。echo_motor_state.shはモータIDの0から4までのモータの状態を表示します。  

# 入出力  
manipulation.py：  

  |用途 |入力topic名 |入力内容 |出力topic名 |出力内容 |  
  |:----------:|:----------:|:-----------:|:----------:|:----------:|  
  |物体把持|/object/grasp_req|String(物体名)|/object/grasp_res|Bool(True)|  
  |アーム変形(carry)|/arm/changing_pose_req|String(carry)|✕|✕|  
  |アーム変形(give)|/arm/changing_pose_req|String(give)|/arm/changing_pose_res|Bool(True)|  
  |アーム変形(place)|/arm/changing_pose_req|String(place)|/arm/changing_pose_res|Bool(True)|  

# その他  
モータIDは左肩、右肩、肘、手首、手先の順に0〜4と割り振られています。また、頭部分は6になっています。  
物体把持は把持が完了するまで何度もリトライします。  

# 注意点  
物体把持を行う場合は頭の角度を-0.07に設定して下さい。  
アームの分解等で原点がズレる場合があるため取り外した際は原点を確認すること  
アームが地面に対して水平になる角度をモータの原点とします。  

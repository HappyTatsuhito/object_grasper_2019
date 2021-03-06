# Overview  
RoboCup2019に向けて開発したマニピュレータを制御するパッケージ  
物体把持、物体設置、モータ調整を行えます。  

# Usage  
モータを制御できるようにセットアップを行います。  

    $ roslaunch object_grasper motor_setup.launch  
アームの動作実験、モータの調整を行う場合は  

    $ rosrun object_grasper echo_motor_state.sh  
	$ rosrun object_grasper experiment_motor.py  
を実行して下さい。echo_motor_state.shはモータIDの0から4までのモータの状態を表示します。  

物体把持を行う場合は  

    $ rosrun object_grasper object_grasper.py  
を実行して下さい。(他パッケージは省略)  

タスクに合ったマニピュレーションノードを立ち上げて下さい。  
大抵のタスクは

    $ rosrun object_grasper general_grasper.py  
で物体把持を行えます。  

# Input and Output  
### object_grasper.py：  

  |動作 |入力topic名 |入力内容 |出力topic名 |出力内容 |  
  |:----------:|:----------:|:-----------:|:----------:|:----------:|  
  |物体把持|/object/grasp_req|String(物体名)|/object/grasp_res|Bool(True)|  
  |アーム変形(carry)|/arm/changing_pose_req|String(carry)|✕|✕|  
  |アーム変形(give)|/arm/changing_pose_req|String(give)|/arm/changing_pose_res|Bool(True)|  
  |アーム変形(place)|/arm/changing_pose_req|String(place)|/arm/changing_pose_res|Bool(True)|  

### experiment_motor.py:  

  |動作 |topic名 |入力内容 |  
  |:----------:|:----------:|:-----------:|  
  |全てのモータをゼロ点に戻す(ID:0,1,2,3,4)|/origin_initialize_req|Bool(True)|  
  |肩の動作確認(ID:0,1)|/shoulder_req|Float64(-3.0〜+2.0)|  
  |肘の動作確認(ID:2)|/elbow_req|Float64(-2.0〜+3.0)|  
  |手首の動作確認(ID:3)|/wrist_req|Float64(-1.5〜+1.5)|  
  |手先を閉じる(ID:4)|/endeffector_req|Bool(True)|  
  

# Other  
モータIDは左肩、右肩、肘、手首、手先の順に0〜4と割り振られています。また、頭部分は6になっています。  
物体把持は把持が完了するまで何度もリトライします。  

# Caution  
物体把持を行う場合は頭の角度を-0.07に設定して下さい。  
アームの分解等で原点がズレる場合があるため取り外した際は原点を確認すること  
アームが地面に対して水平になる角度をモータの原点とします。  

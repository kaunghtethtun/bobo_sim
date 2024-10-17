### Driver
#### friction

#### Plus or Minus One meter test
ဘယ်လိုပဲ friction ထည့်သော်လည်းပဲ hardware robot သို့မဟုတ် simulation robot ရဲ့ 1 meter distance သည်  differential drive kinematic ရဲ့ tf 1 meter နဲ့ အကွာအဝေး အတိအကျ မတူပါဘူး။
- 20 Hz သုံးထားတာလည်း သတိပြုသင့်တယ်။
နမူနာ ယူဖို့အတွက် တရုတ်ရဲ့ reeman robot ကို စစ်ဆေးတဲ့အခါ အောက်ပါအတိုင်းတွေ့ရတယ်။
```md
./install/include/teb_local_planner/teb_config.h:    double weight_max_vel_theta; //!< Optimization weight for satisfying the maximum allowed angular velocity
./install/include/teb_local_planner/teb_config.h:    robot.max_vel_x = 0.4;
./install/include/teb_local_planner/teb_config.h:    robot.max_vel_x_backwards = 0.2;
./install/include/teb_local_planner/teb_config.h:    robot.max_vel_y = 0.0;
./install/include/teb_local_planner/teb_config.h:    robot.max_vel_trans = 0.0;
./install/include/teb_local_planner/teb_config.h:    robot.max_vel_theta = 0.3;
./install/include/teb_local_planner/teb_config.h:    obstacles.obstacle_proximity_ratio_max_vel = 1;


./install/share/yoyo_bringup/param/local_planner_params_robot4.yaml:max_vel_x: 0.5
./install/share/yoyo_bringup/param/local_planner_params_robot4.yaml:max_vel_x_backwards: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot4.yaml:max_vel_y: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot4.yaml:max_vel_theta: 0.5
./install/share/yoyo_bringup/param/local_planner_params_robot2.yaml:max_vel_x: 0.15
./install/share/yoyo_bringup/param/local_planner_params_robot2.yaml:max_vel_x_backwards: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot2.yaml:max_vel_y: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot2.yaml:max_vel_theta: 0.15
./install/share/yoyo_bringup/param/local_planner_params_robot3.yaml:max_vel_x: 0.5
./install/share/yoyo_bringup/param/local_planner_params_robot3.yaml:max_vel_x_backwards: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot3.yaml:max_vel_y: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot3.yaml:max_vel_theta: 0.5
./install/share/yoyo_bringup/param/local_planner_params_robot1.yaml:max_vel_x: 0.5
./install/share/yoyo_bringup/param/local_planner_params_robot1.yaml:max_vel_x_backwards: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot1.yaml:max_vel_y: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot1.yaml:max_vel_theta: 0.5
./install/share/yoyo_bringup/param/local_planner_params_robot5.yaml:max_vel_x: 0.5
./install/share/yoyo_bringup/param/local_planner_params_robot5.yaml:max_vel_x_backwards: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot5.yaml:max_vel_y: 0.0
./install/share/yoyo_bringup/param/local_planner_params_robot5.yaml:max_vel_theta: 0.5
./install/share/yoyo_bringup/param/teb_local_planner_params_common.yaml:  # max_vel_x: 0.5 🌟
./install/share/yoyo_bringup/param/teb_local_planner_params_common.yaml:  # max_vel_x_backwards: 0.0 🌟
./install/share/yoyo_bringup/param/teb_local_planner_params_common.yaml:  # max_vel_y: 0.0
./install/share/yoyo_bringup/param/teb_local_planner_params_common.yaml:  # max_vel_theta: 0.5 🌟
./install/share/yoyo_bringup/param/teb_local_planner_params_common.yaml:  weight_max_vel_x: 2
./install/share/yoyo_bringup/param/teb_local_planner_params_common.yaml:  weight_max_vel_y: 1
./install/share/yoyo_bringup/param/teb_local_planner_params_common.yaml:  weight_max_vel_theta: 1
```
ဆိုတော့ ကာ သူတို့သည် max velocity 0.5 meter persecond နဲ့ backward အတွက် 0.0 ထားထားတယ်။ နောက်ပြန်မသွားခိုင်းတဲ့သဘောပေါ့။
-  ဆိုတော့ ကျတော်တို့လည်းပဲ min velocity = 0.0 နဲ့ 🌟 max velocity 0.5 သုံးမယ်။ 
-  1 meter test လုပ်တဲ့အခါ 0.5 သုံးမလား 0.25 သုံးမလား 0.3 လား 0.4 လားဆိုတော့ 🔥 0.25 ms နဲ့ test မယ်ဗျာ။
-  ဒါပေမဲ့ ကျတော်တို့ Hz သို့မဟုတ် အချိန်နဲ့ မစစ်ဆေးဘဲ simulation သို့ မဟုတ် real environment မှာ 1 meter သွားခိုင်းပြီး rviz မှာ tf 1 meter ပြည့်မပြည့် စစ်ဆေးပါမယ်။
- plus ပြီးရင် minus စမ်းချင်စမ်းလို့ရပါတယ်။

#### Plus or Minus 2pi radian test
- max_vel_theta က 0.5 radian per second ထားတော့ ကျတော်တို့လည်း 🌟 max_angular_velocity = 0.5 ထားပြီး 2Pi test ကတော့ 🔥 0.25 rds နဲ့ စမ်းမယ်ဗျာ။
- robot ကို angular 0.25 rds * 20 Hz ဆိုရင် 5 radian ရောက်ရပါမယ်။ ဒါပေမဲ့ ကျတော်တို့ အချိန်နဲ့ မခိုင်းဘဲ 2PI radian ပြည့်အောင် publish လုပ်ပေးပါမယ်။
- plus ပြီးရင် minus စမ်းကို စမ်းရပါမယ်။



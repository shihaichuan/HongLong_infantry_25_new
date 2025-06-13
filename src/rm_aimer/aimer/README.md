# rm_aimer
rm_aimer
        |——aimer
                |——config
                |——include
                |——src
                      |——Aimer.cpp——————建立观测模型，发布观测数据
                      |——KinematicModel.cpp
                      |——ModelObserver.cpp——————根据模型进行观测
## 节点说明
接收armor_detector节点下面的/detector/armors话题消息，获取Armor.msg消息，包括id, yaw_angle, pose, type等。再根据装甲板类型以及id进行建模预测,若是小装甲板,并且id为1或者2（英雄、非平衡步兵与哨兵）则建立Standard模型,若id为3，4，5则建立Balance模型。建立模型后进入ModelObserver进行观测建模。


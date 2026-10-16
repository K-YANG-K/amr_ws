# amr_ws导航说明

## 一、涉及的概念

### 1.action

![image-20260610091039660](/home/kane/.config/Typora/typora-user-images/image-20260610091039660.png)

通过3+2来实现长时服务

/amr_bt_navigator/amr_nav_mode_a/_action/cancel_goal
/amr_bt_navigator/amr_nav_mode_a/_action/get_result
/amr_bt_navigator/amr_nav_mode_a/_action/send_goal

/amr_bt_navigator/amr_nav_mode_a/_action/feedback
/amr_bt_navigator/amr_nav_mode_a/_action/status

	int8 STATUS_UNKNOWN   = 0
	int8 STATUS_ACCEPTED  = 1
	int8 STATUS_EXECUTING = 2
	int8 STATUS_CANCELING = 3
	int8 STATUS_SUCCEEDED = 4
	int8 STATUS_CANCELED  = 5
	int8 STATUS_ABORTED   = 6

### 2.lifecycle

![image-20260610095115623](/home/kane/.config/Typora/typora-user-images/image-20260610095115623.png)

on_configure()
on_activate()
on_deactivate()
on_cleanup()
on_shutdown()

所有受控节点通过lifecycle manager来统一管理

lifecycle_msgs/srv/ChangeState

lifecycle_msgs/srv/GetState

### 3.component

Component 是 ROS2 的节点组件化机制，把原本独立进程运行的 Node 编译成动态库（.so），由 component_container 在同一进程中动态加载运行，从而减少进程数量、降低 通信开销

### 4.行为树

1.关于blackboard要修改的地方

以导航A的最外层action为切入点

1)AmrNavModeA.action

```
#goal

nav_msgs/Path nav_a_goal

std_msgs/Float32 nav_speed
```

2)改导航A对应的类src/amr_navigation/amr_bt_navigator/src/navigators/amr_nav_mode_a.cpp

```
blackboard->set<nav_msgs::msg::Path>(goal_blackboard_id_, goal->nav_a_goal);

blackboard->set<std_msgs::msg::Float32>(speed_blackboard_id_, goal->nav_speed);
```

其中speed_blackboard_id_为黑板的key值，也通过logistics_nav_params.yaml中的参数来修改，

这一步就是将action的goal通过key写到黑板中

3)子节点获取相应的值

src/amr_navigation/amr_behavior_tree/include/amr_behavior_tree/plugins/action/generate_follow_path_action.hpp

```c++
    static BT::PortsList providedPorts()

​    {

​        return providedBasicPorts(

​        {

​            BT::InputPort<nav_msgs::msg::Path>("nav_a_goal", "from tcs goal path"),

​            BT::InputPort<std::string>("planner_id", "NavAPlanner", "planner plugin name"),

​            BT::InputPort<std_msgs::msg::Float32>("nav_speed","nav a speed"),

​            BT::OutputPort<nav_msgs::msg::Path>("global_path", "Path created by GenerateFollowPath node"),

​        });

​    }
```

各个子节点通过providedPorts注册行为树xml的端口name

```
 InputPort<Path>("nav_a_goal", ...)  ←→  nav_a_goal="{nav_a_goal}"
```

后面等号左边的是port的名字，{}里的是通过key获取的值

src/amr_navigation/amr_behavior_tree/plugins/action/generate_follow_path_action.cpp

```
void GenerateFollowPathAction::on_tick(){

​    getInput("nav_a_goal",goal_.nav_a_goal);

​    getInput("planner_id",goal_.planner_id);

​    getInput("nav_speed",goal_.nav_speed);

}
```

最后通过代码中"nav_a_goal"（port的name，name又是通过xml里nav_a_goal="{nav_a_goal}"来获取的），把数据从黑板搬到 action goal 里，准备发送给下层服务端。结果的set同理

4）src/amr_navigation/amr_behavior_tree/amr_tree_nodes.xml是给Groot看的



2.关于子节点action的result通过blackboard透传到最外层action（以路径生成为例）

1)在src/amr_navigation/amr_msgs/action/GenerateFollowPath.action中声明相应的状态码

2）在src/amr_navigation/amr_planner/src/planner_server.cpp中要设置result的地方进行赋值

```
      result->nav_status = ActionGenerateFollowPath::Result::GENERATE_ERROR;

​      action_server_generate_follow_path_->terminate_current(result);
```

3）在对应的行为是客户端进行端口name声明

src/amr_navigation/amr_behavior_tree/include/amr_behavior_tree/plugins/action/generate_follow_path_action.hpp

以及在src/amr_navigation/amr_behavior_tree/plugins/action/generate_follow_path_action.cpp中进行键值对赋值，将结果写入黑板

```
BT::NodeStatus GenerateFollowPathAction::on_success(){

​    setOutput("global_path",result_.result->global_path);

​    setOutput("nav_status", result_.result->nav_status);

​    return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus GenerateFollowPathAction::on_aborted(){

​    nav_msgs::msg::Path empty_path;

​    setOutput("nav_a_goal",empty_path);

​    setOutput("nav_status", result_.result->nav_status);

​    return BT::NodeStatus::FAILURE;

}
```

4）最后在外层服务端读黑板并赋值给最终的result

```
    void AmrNavModeANavigator::goalCompleted(typename ActionT::Result::SharedPtr result, const amr_behavior_tree::BtStatus final_bt_status){

​        auto blackboard = bt_action_server_->getBlackboard();

​        uint16_t status = 0;

​        blackboard->get<uint16_t>("nav_status", status);

​        LOG(INFO) << "goalCompleted: blackboard nav_status = " << status

​                  << ", bt_status = " << static_cast<int>(final_bt_status);

​        result->nav_status = status;

​    }
```


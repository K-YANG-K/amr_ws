#!/bin/bash
ros2 action send_goal /amr_bt_navigator/amr_nav_mode_b \
amr_msgs/action/AmrNavModeB \
'{
  nav_b_goal: {
    header: {
      frame_id: "map"
    },
    pose: {
      position: {
        x: 0.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  },
  nav_speed: {
    data: 2.0
  }
}'
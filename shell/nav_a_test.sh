#!/bin/bash
# ros2 action send_goal /amr_bt_navigator/amr_nav_mode_a amr_msgs/action/AmrNavModeA \
# '{
#   nav_a_goal: [
#     {
#       header: {
#         frame_id: "map"
#       },
#       pose: {
#         position: {
#           x: 0.0,
#           y: 0.0,
#           z: 0.0
#         },
#         orientation: {
#           x: 0.0,
#           y: 0.0,
#           z: 0.0,
#           w: 1.0
#         }
#       }
#     },
#     {
#       header: {
#         frame_id: "map"
#       },
#       pose: {
#         position: {
#           x: 3.0,
#           y: 0.0,
#           z: 0.0
#         },
#         orientation: {
#           x: 0.0,
#           y: 0.0,
#           z: 0.0,
#           w: 1.0
#         }
#       }
#     }
#   ],
#   nav_speed: {
#     data: 10.51456462
#   }
# }'


#贝塞尔

ros2 action send_goal /amr_bt_navigator/amr_nav_mode_a \
amr_msgs/action/AmrNavModeA \
'{
  nav_a_goal: [
    {
      header: {
        frame_id: "map"
      },
      pose: {
        position: {x: 1.0, y: 0.0, z: 10000.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {
        frame_id: "map"
      },
      pose: {
        position: {x: 2.2, y: 0.8, z: 10000.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {
        frame_id: "map"
      },
      pose: {
        position: {x: 2.5, y: 1.5, z: 10000.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {
        frame_id: "map"
      },
      pose: {
        position: {x: 1.0, y: 2.0, z: 10000.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }
  ],
  nav_speed: {
    data: 10.52
  }
}'
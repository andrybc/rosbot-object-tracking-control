# rosbot-object-tracking-control

Vendor Yahboom object/face tracking (Python) plus custom control (C++) for:
- camera pan/tilt (servo_s1, servo_s2)
- slow base yaw (cmd_vel)

Goal: reuse vendor detection/tracking, then modify it to publish tracking errors; C++ node will control servos/base.

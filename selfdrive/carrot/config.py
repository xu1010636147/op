#!/usr/bin/env python3
"""
配置管理模块
负责系统配置参数的初始化和持久化存储
"""

import json
from common.params import Params

class Config:
    """配置管理类"""
    
    def __init__(self):
        """初始化配置参数"""
        self.params = Params()
        self.config = self._init_default_config()
        self.load_persistent_config()
    
    def _init_default_config(self):
        """初始化默认配置"""
        return {
            # 道路和车道配置
            'road_type': 'highway',
            'lane_count': 3,
            'current_lane_number': 2,
            'lane_count_mode': 'auto',
            'manual_lane_count': 3,

            # 超车功能开关
            'autoOvertakeEnabled': False,
            'autoOvertakeEnabledL': False,
            'shouldReturnToLane': True,

            # 前车最低速度限制
            'HIGHWAY_LEAD_MIN_SPEED': 35.0,
            'NORMAL_LEAD_MIN_SPEED': 20.0,

            # 超车触发条件参数
            'HIGHWAY_MIN_SPEED': 75.0,
            'NORMAL_ROAD_MIN_SPEED': 40.0,
            'CRUISE_SPEED_RATIO_THRESHOLD': 0.8,
            'FOLLOW_TIME_GAP_THRESHOLD': 2.0,
            'MAX_FOLLOW_TIME': 600000,
            'LEAD_RELATIVE_SPEED_THRESHOLD': -15.0,

            # 远距离超车参数
            'EARLY_OVERTAKE_SPEED_RATIO': 0.6,
            'EARLY_OVERTAKE_MIN_LEAD_SPEED': 50.0,
            'EARLY_OVERTAKE_MIN_DISTANCE': 30.0,
            'EARLY_OVERTAKE_MAX_DISTANCE': 100.0,
            'EARLY_OVERTAKE_MIN_SPEED_DIFF': 20.0,

            # 安全变道条件参数
            'MIN_LANE_WIDTH': 2.3,
            'SAFE_LANE_WIDTH': 2.8,
            'SIDE_LEAD_DISTANCE_MIN': 25.0,
            'SIDE_RELATIVE_SPEED_THRESHOLD': 25,

            # 弯道检测参数
            'CURVATURE_THRESHOLD': 0.02,
            'STEERING_THRESHOLD': 10.0,

            # 冷却时间参数(毫秒)
            'OVERTAKE_COOLDOWN_BASE': 8000,
            'OVERTAKE_COOLDOWN_FAILED': 3000,
            'OVERTAKE_COOLDOWN_SUCCESS': 15000,
            'OVERTAKE_COOLDOWN_CONDITION': 5000,

            # 惩罚权重系统
            'PENALTY_WEIGHTS': {
                'lead_relative_speed': 2.0,
                'side_lead_distance': 1.5,
                'side_relative_speed': 1.8,
                'lane_width': 1.2,
                'blindspot': 3.0,
                'curvature': 1.5,
                'min_speed_advantage': 5.0
            },

            # 决策阈值
            'PENALTY_THRESHOLD': 60.0,
            'MIN_SPEED_ADVANTAGE': 5.0,

            # 返回策略配置
            'RETURN_STRATEGY': {
                'highway': {
                    'enabled': True,
                    'return_timeout': 30000,
                    'max_return_attempts': 3,
                },
                'normal': {
                    'enabled': False,
                    'return_timeout': 0,
                    'max_return_attempts': 0,
                }
            },

            # 高速公路专用策略
            'HIGHWAY_STRATEGY': {
                'prefer_left_overtake': True,
                'avoid_rightmost_lane': True,
                'emergency_lane_penalty': 100,
                'fast_lane_bonus': 15,
                'min_advantage_threshold': 3
            }
        }

    def load_persistent_config(self):
        """从持久化存储加载配置"""
        try:
            config_json = self.params.get("AutoOvertakeConfig")
            if config_json is not None:
                saved_config = json.loads(config_json)
                print(f"📥 加载保存的配置")
                # 只更新已存在的配置项
                for key, value in saved_config.items():
                    if key in self.config:
                        self.config[key] = value
            else:
                print("📥 使用默认配置")
        except Exception as e:
            print(f"⚠️ 加载配置失败: {e}")

    def save_persistent_config(self):
        """保存配置到持久化存储"""
        try:
            self.params.put("AutoOvertakeConfig", json.dumps(self.config))
            print("✅ 配置已保存")
        except Exception as e:
            print(f"⚠️ 保存配置失败: {e}")

    def get(self, key, default=None):
        """获取配置值"""
        return self.config.get(key, default)

    def set(self, key, value):
        """设置配置值"""
        self.config[key] = value

    def update(self, updates):
        """批量更新配置"""
        for key, value in updates.items():
            if key in self.config:
                self.config[key] = value
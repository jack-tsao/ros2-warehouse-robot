import streamlit as st
from streamlit_modal import Modal
import pandas as pd
import time
import os
import base64
import json
from datetime import datetime, timedelta
import signal
from streamlit_autorefresh import st_autorefresh
from streamlit_chat import message
import subprocess
from PIL import Image, UnidentifiedImageError
import rclpy
from rclpy.node import Node
from adv_msgs.msg import AdvSensor
import threading
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from transforms3d.euler import euler2quat
from sensor_msgs.msg import Image as RosImage
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from ultralytics import YOLO
import csv
import zmq
from collections import Counter
import altair as alt
import plotly.graph_objects as go
import plotly.express as px
from streamlit_chat import message
from openai import OpenAI
import streamlit.components.v1 as components
from streamlit_javascript import st_javascript


# 語言文字表
TEXT = {
    "繁體中文": {
        "power_title": "⏹️ 關閉電源",
        "power_caption": "點擊下方按鈕可關閉機器。請確保資料已儲存完畢！",
        "power_button": "🔌 關閉機器",
        "power_warning": "⚠️ 準備關機中...",
        "power_error": "⚠️ 執行關機指令時發生錯誤：",

        "reboot_title": "🔁 重新啟動機器",
        "reboot_caption": "執行此操作將立即重新啟動系統，請務必確認所有資料已儲存。",
        "reboot_button": "🔁 立即重新啟動",
        "reboot_warning": "⚠️ 系統即將重新啟動，請儘速保存工作內容。",
        "reboot_error": "❌ 重啟失敗：",

        "language_title": "🌐 語言設定",
        "language_caption": "請選擇介面語言，將即時套用在系統中。",
        "language_radio": "選擇語言：",
        "language_success": "✅ 目前介面語言：",

        "theme_title": "🎨 顏色主題設定",
        "theme_caption": "請選擇系統主題顏色，會影響整體風格。",
        "theme_radio": "選擇主題：",
        "theme_success": "🎨 目前主題：",

        "contact_title": "📞 緊急聯絡人",
        "contact_caption": "如遇機器異常、無法控制或發生安全事件，請立即聯絡以下人員：",

        "logout_title": "🔓 登出帳號",
        "logout_caption": "點擊下方按鈕可登出目前帳號並返回主頁。",
        "logout_button": "🚪 登出",
        "logout_success": "👋 已成功登出！",

        "sidebar_title": "🤖 AMR 控制選單",
        "welcome_user": "👋 歡迎訪問：",
        "honorific": "先生/小姐",
        "sidebar_pages": ["🏠 主頁", "📡 啟動雷達", "📍 導航任務", "📷 相機畫面", "📊 機器狀態", "🗂️ 任務日誌", "⚙️ 系統設定"],
        "nav_modes": ["📍 座標導航", "💬 語義導航"],
        "login": "👤 登入",
        "register": "📝 註冊",
        "login_modal_title": "🔐 使用者登入",
        "register_modal_title": "🆕 註冊帳號",
        "login_success": "✅ 歡迎回來！",
        "login_error": "❌ 帳號或密碼錯誤",
        "register_success": "✅ 註冊成功！",
        "login_account": "帳號",
        "login_password": "密碼",
        "register_account": "設定帳號",
        "register_password": "設定密碼",
        "register_password2": "再次輸入密碼",
        "login_button": "登入",
        "register_button": "註冊",
        "error_login_required": "⚠️ 請先登入以使用此功能，已自動返回主頁。",

        "hardware_title": "🧠 硬體平台資訊 （ Advantech ARK-3533）",
        "hardware_spec_item": "項目",
        "hardware_spec_description": "規格說明",
        "hardware_cpu": "🖥️ 處理器 (CPU)",
        "hardware_cpu_desc": "第12代 Intel Core i9 / i7 / i5 / i3（Alder Lake）",
        "hardware_gpu": "🎮 GPU 擴充",
        "hardware_gpu_desc": "支援 PCIe x16 擴充外部 GPU（NVIDIA 等）",
        "hardware_ram": "🧠 記憶體 (RAM)",
        "hardware_ram_desc": "DDR5 支援，最大 64GB（SODIMM ×2）",
        "hardware_storage": "💾 儲存",
        "hardware_storage_desc": "M.2 NVMe ×1、SATA SSD/HDD ×2",
        "hardware_network": "🌐 網路",
        "hardware_network_desc": "2× GbE LAN，Wi-Fi / 5G / LTE 支援",
        "hardware_io": "🔌 I/O",
        "hardware_io_desc": "USB 3.2、HDMI / DP、COM、CANBus",
        "hardware_temp": "🌡️ 工作溫度",
        "hardware_temp_desc": "-20°C ~ 60°C，無風扇設計",
        "hardware_expansion": "🧩 擴充性",
        "hardware_expansion_desc": "M.2、mini PCIe、SIM 卡槽",
        "hardware_spec_local": "本機信息",
        "hardware_expansion_local": "M.2 ×2（含NVMe）、Mini PCIe ×2、PCIe多槽、SATA、SIM 卡槽",
        "chatbot_title": "💻 AMR 小幫手 ChatBot",
        "chatbot_input": "💬 你想說什麼？",
        "chatbot_clear": "🧹 清空對話",
        "chatbot_error": "⚠️ 發生錯誤：",
        "chatbot_system_prompt": "你是 AMR 小幫手，擅長機器人導航、避障、狀態回報等任務。",

        "radar_title_1": "📡 1. 啟動雷達",
        "radar_info_1": "請打開終端，並輸入以下兩行指令：",
        "radar_input_command": "輸入 Linux 指令:",
        "radar_execute": "🚀 執行",
        "radar_success": "（✅ 成功啟動雷達（背景執行中））",
        "radar_input_warning": "請輸入指令",
        "radar_title_2": "🖼️ 2. Rviz2 雷達地圖顯示區",
        "radar_show_button": "🛰️ 顯示雷達圖",
        "radar_show_success": "✅ 已開始從 RViz 持續截圖...",
        "radar_stop_button": "❌ 暫停更新雷達圖",
        "radar_stop_success": "🛑 已關暫停更新 RViz",
        "radar_image_caption": "📡 雷達圖即時顯示中",
        "radar_no_image": "⚠️ 尚未偵測到最新的雷達圖",
        "radar_title_3": "💾 3. 保存地圖 / 關閉雷達",
        "radar_info_3": "請輸入以下兩行指令",
        "radar_no_output": "（無輸出）",

        "env_init_title": "🔧 1. 環境初始化",
        "env_init_info": "請輸入以下指令",
        "input_command": "輸入 Linux 指令:",
        "execute_button": "🚀 執行",
        "input_warning": "⚠️ 請輸入指令！",
        "execute_success": "✅ 指令已在背景執行，PID：{}",
        "execute_error": "❌ 執行時發生錯誤：{}",

        "nav_task_title": "🧭 2. 設定導航任務",
        "start_nav_node": "🤖 啟動導航節點",
        "nav_node_success": "✅ 導航節點已啟動",
        "nav_node_info": "ℹ️ 導航節點已經在運行中或啟動失敗",
        "nav_dialog_title": "🧭 設定導航任務",
        "open_nav_dialog": "➕ 開啟導航任務彈窗",

        "start_coord_title": "🟢 輸入初始座標點",
        "start_x": "📍 起點 X",
        "start_y": "📍 起點 Y", 
        "start_yaw": "🧭 起點角度（Yaw）",
        "set_start_button": "✅ 設定起點",
        "start_set_success": "✅ 起點已設定並發送：X={}, Y={}, 起點角度={:.2f}",
        "ros_node_warning": "⚠️ 尚未初始化 ROS 節點，無法發送初始位置",

        "goal_coord_title": "🎯 輸入目標座標點",
        "goal_group": "第 {} 組",
        "goal_x": "🎯 X {}",
        "goal_y": "🎯 Y {}",
        "goal_yaw": "🧭 Yaw {}",
        "add_goal_button": "➕ 新增一組目標點",
        "send_nav_button": "✅ 發送導航任務",
        "ros_node_error": "❌ 尚未初始化 ROS 節點",
        "nav_task_success": "📤 目標點已送出並啟動導航",

        "control_panel": "⚙️ 控制面板",
        "nav_step1_title": "🟦 步驟 1：啟動 Rviz2",
        "nav_step1_button": "啟動 Rviz2",
        "nav_step1_loading": "正在啟動 Rviz2，請稍候...",
        "nav_step1_success": "✅ Rviz2 已啟動！",
        "nav_step1_error": "❌ 啟動失敗：{error}",
        "nav_step2_title": "🟩 步驟 2：啟動語義導航節點",
        "nav_step2_button": "啟動語義導航節點",
        "nav_step2_loading": "正在啟動語義導航節點，請稍候...",
        "nav_step2_success": "✅ Granite + BLIP + YOLO 模型載入完成",
        "nav_step2_warning": "⚠️ 模型啟動中，請稍候幾秒後再送出任務或查看 log 狀態。",
        "nav_step2_error": "❌ 啟動失敗：{error}",
        "nav_step3_title": "🟩 步驟 3：設置初始坐標點",
        "nav_step3_button": "設置初始坐標點",
        "nav_step3_loading": "正在發送初始位姿 (0, 0, 0) 至 Rviz2...",
        "nav_step3_success": "📍 已成功發佈初始位姿 (0, 0, 0)",
        "nav_step3_error": "❌ 發送初始位姿失敗：{error}",
        "nav_step4_title": "🟨 步驟 4：選擇模型",
        "nav_step4_select": "選擇模型類型：",
        "nav_step4_success": "✅ 已選擇模型：{model}",
        "nav_step5_title": "🟦 步驟 5：輸入任務",
        "nav_step5_placeholder": "請輸入任務描述…",
        "nav_step5_button": "送出任務",
        "nav_step5_success": "🧠 任務已送出：{task}",
        "nav_step5_warning": "⚠️ 請先輸入任務內容！",
        "nav_step5_error": "❌ 發送失敗：{error}",
        "nav_step6_title": "🟥 步驟 6：結束導航並關閉 Rviz2",
        "nav_step6_button": "結束導航並關閉 Rviz2",
        "nav_step6_loading": "正在關閉 Rviz2 與語義導航節點...",
        "nav_step6_success": "🛑 已成功執行 stop_navigation.sh 並關閉 Rviz2！",
        "nav_step6_warning": "⚠️ stop_navigation.sh 執行完成，但可能有警告：{result.stderr}",
        "nav_step6_error": "❌ 關閉失敗：{e}",

        "nav_task_order": "📜 任務執行順序",
        "model_response": "✨ 模型回傳結果",

        "nav_status_title": "🚗 3. 機器人導航狀態總覽",
        "total_goals": "📋 **總目標數量：** {} 個",
        "current_goal_ready": "📍 **當前目標：** 🚦 準備開始",
        "current_goal_progress": "📍 **當前目標：** 🎯 第 {} / {} 個",
        "current_goal_return": "📍 **當前目標：** 🏠 返回起點",
        "nav_status_label": "🧱 導航狀態：",
        "yolo_title": "🧠 YOLO 偵測結果",
        "yolo_no_detection": "📷 尚未偵測到任何物體",

        "status_running": "進行中",
        "status_avoiding": "避障中", 
        "status_paused": "暫停中",

        "seg_title": "🧠 即時語義分割畫面",
        "semantic_caption": "語義分割結果",
        "waiting_seg": "⏳ 等待語義分割影像...",
        "ros_not_ready": "⚠️ ROS 節點尚未初始化，無法顯示語義分割影像",

        "end_task_title": "🔚 4. 結束任務",
        "close_nav_button": "🛑 關閉導航",
        "close_nav_success": "✅ 導航與 YOLO 關閉成功",
        "no_output": "✅ 無標準輸出",
        "error_output": "❌ 錯誤輸出：\n{}\n{}",
        "execute_failed": "❌ 執行失敗：{}",

        "camera_env_init": "環境初始化",
        "camera_env_info": "請輸入以下指令",
        "camera_input_command": "輸入 Linux 指令",
        "camera_execute": "執行",
        "camera_command_success": "指令已在背景執行，PID：",
        "camera_error": "發生錯誤：",
        "camera_input_python": "輸入 Python 指令",
        "camera_python_success": "Python 指令已在背景執行，PID：",
        "camera_view_title": "查看攝像頭畫面",
        "camera_view_caption": "點擊按鈕開啟或關閉攝像頭，顯示即時畫面。",
        "camera_start": "開啟攝像頭",
        "camera_stop": "關閉攝像頭",
        "camera_script_closed": "open_camera.sh 已強制關閉",
        "camera_script_close_error": "關閉 open_camera.sh 時發生錯誤：",
        "camera_python_closed": "testCameraWeb.py 已強制關閉",
        "camera_python_close_error": "關閉 testCameraWeb.py 時發生錯誤：",
        "camera_status": "狀態：",
        "camera_status_on": "已開啟",
        "camera_status_off": "已關閉",
        "camera_image_caption": "Robot Camera",
        "camera_no_image": "尚未偵測到相機圖片",
        "camera_control_title": "鍵盤控制機器人行走",
        "camera_control_caption": "使用下方按鈕控制機器人前進 / 後退 / 左轉 / 右轉",
        "camera_send_command": "發送指令",
        "camera_speed_setting": "速度設定",
        "camera_linear_speed": "線速度（前後）",
        "camera_angular_speed": "角速度（旋轉）",
        "camera_keyboard_control": "鍵盤方向控制",
        "camera_forward": "W（前進）",
        "camera_left": "A（左轉）",
        "camera_backward": "S（後退）",
        "camera_right": "D（右轉）",
        "camera_emergency_stop": "緊急停止",
        "camera_stop_robot": "停止機器人",
        "camera_stop_sent": "已發送停止指令",

        "susi_status_title": "機器狀態查看（SUSI Node）",
        "susi_status_info": "請打開終端，並依序輸入以下指令，以初始化 ROS2 與 SUSI 環境：",
        "susi_input_command": "輸入 Linux 指令",
        "susi_start": "啟動 SUSI",
        "susi_stop": "關閉 SUSI",
        "susi_starting": "SUSI 啟動中",
        "susi_exec_error": "執行時發生錯誤：",
        "susi_input_warning": "請輸入完整指令後再執行",
        "susi_stopped": "SUSI 已成功關閉",
        "susi_stop_error": "關閉時發生錯誤：",
        "susi_not_running": "SUSI 尚未啟動或已結束",

        "susi_monitor_title": "SUSI 機器狀態監控",
        "susi_manual_refresh": "手動刷新",
        "susi_auto_refresh": "自動刷新",
        "susi_data_error": "資料讀取錯誤：",
        "susi_missing_time": "缺少資料時間欄位！",
        "susi_data_time": "資料時間：",
        "susi_outdated": "資料過時",
        "susi_latest": "資料最新",
        "susi_monitor_items": "監控項目：",
        "susi_data_outdated_warning": "⚠️ 請執行susi指令獲取最新資料",
        "susi_next_refresh": "下次刷新：",
        "susi_seconds": "秒後",
        "desc_3v": "（供 BIOS / 控制器）",
        "desc_5v": "（USB / 硬碟 / 控制電路）",
        "desc_12v": "（風扇 / 馬達 / 顯卡）",
        "desc_cmos": "（主機板電池）",
        "voltage_monitor_title": "🔋 電壓監控",
        "temperature_monitor_title": "🌡️ 溫度監控",
        "temperature_label": "溫度",
        "fan_monitor_title": "🌀 風扇監控",
        "current_monitor_title": "⚡ 電流監控",
        "disk_monitor_title": "📀 儲存資訊",
        "total_disk_label": "📀 **總磁碟空間**",
        "susi_time_format_error": "資料時間格式解析錯誤：",
        "fan_label": "風扇",
        "cpu_fan_stopped": "CPU風扇停止 - 請立即檢查！",
        "fan_not_running": "風扇未運轉",
        "debug_checkbox_label": "顯示調試資訊",
        "debug_current_time": "當前時間: {time:.1f}",
        "debug_last_update": "上次更新: {time:.1f}",
        "debug_time_diff": "時間差: {seconds:.1f}秒",
        "debug_remaining_time": "剩餘時間: {seconds:.1f}秒",
        "debug_progress": "進度: {progress:.2%}",
        "debug_data_timestamp": "數據時間戳: {timestamp:.1f}",
        "debug_data_update_diff": "數據與更新時間差: {diff:.1f}秒",
        

        "analysis_title": "🧠 資料分析與可視化",
        "analysis_yolo_caption": "📎 YOLO 偵測物件次數統計",
        "analysis_yolo_time_expired": "⚠️ 上次任務已超過 10 分鐘（保存時間：{time}），圖表不顯示。",
        "analysis_yolo_no_time": "⚠️ 找不到保存時間，無法驗證資料是否過期，請確認記錄格式。",
        "analysis_yolo_error": "❌ YOLO 偵測記錄讀取失敗",
        "analysis_path_caption": "📍 實際路徑與規劃路徑",
        "analysis_path_no_csv": "⚠️ 找不到路徑資料 CSV 檔案，請確認機器人是否已儲存路徑。",
        "analysis_path_empty": "⚠️ 路徑資料為空，尚無實際路徑或規劃路徑可供顯示。",
        "analysis_path_title": "Robot Navigation Path",
        "analysis_object_caption": "🔥 物件偵測分佈圖",
        "analysis_object_no_data": "⚠️ 尚無有效的偵測資料",
        "analysis_object_title": "物件位置分佈圖",
        "analysis_time_error": "距離上次記錄超出時間",
        "analysis_object_time_expired": "⚠️ 物件偵測資料已過期（最後更新：{time}）",
        "analysis_path_time_expired": "⚠️ 路徑資料已過期（最後更新：{time}）"
    },
    "日本語": {
        "power_title": "⏹️ 電源オフ",
        "power_caption": "下のボタンをクリックして電源を切ってください。データが保存されていることを確認してください。",
        "power_button": "🔌 シャットダウン",
        "power_warning": "⚠️ シャットダウン中...",
        "power_error": "⚠️ シャットダウンコマンドの実行に失敗しました：",

        "reboot_title": "🔁 再起動",
        "reboot_caption": "この操作を実行すると、すぐにシステムが再起動されます。データが保存されていることをご確認ください。",
        "reboot_button": "🔁 今すぐ再起動",
        "reboot_warning": "⚠️ システムを再起動しています。作業内容を保存してください。",
        "reboot_error": "❌ 再起動に失敗しました.",

        "language_title": "🌐 言語設定",
        "language_caption": "インターフェースの言語を選択してください。すぐに適用されます。",
        "language_radio": "言語を選択：",
        "language_success": "✅ 現在の言語：",

        "theme_title": "🎨 テーマ設定",
        "theme_caption": "システムのテーマカラーを選択してください。全体のスタイルに影響します。",
        "theme_radio": "テーマを選択：",
        "theme_success": "🎨 現在のテーマ：",

        "contact_title": "📞 緊急連絡先",
        "contact_caption": "ロボットの故障、応答なし、安全上の問題が発生した場合は、すぐに以下の担当者に連絡してください。",

        "logout_title": "🔓 ログアウト",
        "logout_caption": "下のボタンをクリックしてログアウトし、ホームページに戻ります。",
        "logout_button": "🚪 ログアウト",
        "logout_success": "👋 正常にログアウトしました！",

        "sidebar_title": "🤖 AMR操作メニュー",
        "welcome_user": "👋 ようこそ：",
        "honorific": "様",
        "sidebar_pages": ["🏠 ホーム", "📡 レーダー起動", "📍 ナビ任務", "📷 カメラ画面", "📊 ロボット状態", "🗂️ 任務ログ", "⚙️ システム設定"],
        "nav_modes": ["📍 座標ナビ", "💬 自然言語ナビ"],
        "login": "👤 ログイン",
        "register": "📝 登録",
        "login_modal_title": "🔐 ログイン",
        "register_modal_title": "🆕 アカウント登録",
        "login_success": "✅ お帰りなさい！",
        "login_error": "❌ ユーザー名またはパスワードが間違っています",
        "register_success": "✅ 登録完了！",
        "login_account": "ユーザー名",
        "login_password": "パスワード",
        "register_account": "ユーザー名を設定",
        "register_password": "パスワードを設定",
        "register_password2": "パスワードもう一度入力",
        "login_button": "ログイン",
        "register_button": "登録",
        "error_login_required": "⚠️ 先にログインしてください。ホームに戻りました。",

        "hardware_title": "🧠 ハードウェアプラットフォーム情報 （ Advantech ARK-3533）",
        "hardware_spec_item": "項目",
        "hardware_spec_description": "仕様説明",
        "hardware_cpu": "🖥️ プロセッサー (CPU)",
        "hardware_cpu_desc": "第12世代 Intel Core i9 / i7 / i5 / i3（Alder Lake）",
        "hardware_gpu": "🎮 GPU拡張",
        "hardware_gpu_desc": "PCIe x16 外部GPU拡張サポート（NVIDIA等）",
        "hardware_ram": "🧠 メモリ (RAM)",
        "hardware_ram_desc": "DDR5対応、最大64GB（SODIMM ×2）",
        "hardware_storage": "💾 ストレージ",
        "hardware_storage_desc": "M.2 NVMe ×1、SATA SSD/HDD ×2",
        "hardware_network": "🌐 ネットワーク",
        "hardware_network_desc": "2× GbE LAN、Wi-Fi / 5G / LTE サポート",
        "hardware_io": "🔌 I/O",
        "hardware_io_desc": "USB 3.2、HDMI / DP、COM、CANBus",
        "hardware_temp": "🌡️ 動作温度",
        "hardware_temp_desc": "-20°C ~ 60°C、ファンレス設計",
        "hardware_expansion": "🧩 拡張性",
        "hardware_expansion_desc": "M.2、mini PCIe、SIMカードスロット",
        "hardware_spec_local": "本体情報",
        "hardware_expansion_local": "M.2 ×2（NVMe含む）、Mini PCIe ×2、複数のPCIeスロット、SATA、SIMカードスロット",
        "chatbot_title": "💻 AMR アシスタント ChatBot",
        "chatbot_input": "💬 何か質問はありますか？",
        "chatbot_clear": "🧹 会話をクリア",
        "chatbot_error": "⚠️ エラーが発生しました：",
        "chatbot_system_prompt": "あなたは AMR アシスタントで、ロボットナビゲーション、障害物回避、状態報告などのタスクが得意です。",

        "radar_title_1": "📡 1. レーダー起動",
        "radar_info_1": "ターミナルを開いて、以下の2行のコマンドを入力してください：",
        "radar_input_command": "Linuxコマンドを入力:",
        "radar_execute": "🚀 実行",
        "radar_success": "（✅ レーダーが正常に起動しました（バックグラウンドで実行中））",
        "radar_input_warning": "コマンドを入力してください",
        "radar_title_2": "🖼️ 2. Rviz2 レーダーマップ表示エリア",
        "radar_show_button": "🛰️ レーダー図を表示",
        "radar_show_success": "✅ RVizからの継続的なスクリーンショットを開始しました...",
        "radar_stop_button": "❌ レーダー図の更新を停止",
        "radar_stop_success": "🛑 RVizの更新を停止しました",
        "radar_image_caption": "📡 レーダー図をリアルタイム表示中",
        "radar_no_image": "⚠️ 最新のレーダー図がまだ検出されていません",
        "radar_title_3": "💾 3. マップ保存 / レーダー終了",
        "radar_info_3": "以下の2行のコマンドを入力してください",
        "radar_no_output": "（出力なし）",

        "env_init_title": "🔧 1. 環境初期化",
        "env_init_info": "以下のコマンドを入力してください",
        "input_command": "Linuxコマンドを入力:",
        "execute_button": "🚀 実行",
        "input_warning": "⚠️ コマンドを入力してください！",
        "execute_success": "✅ コマンドをバックグラウンドで実行、PID：{}",
        "execute_error": "❌ 実行時にエラーが発生しました：{}",

        "nav_task_title": "🧭 2. ナビゲーションタスク設定",
        "start_nav_node": "🤖 ナビゲーションノード開始",
        "nav_node_success": "✅ ナビゲーションノードが開始されました",
        "nav_node_info": "ℹ️ ナビゲーションノードは既に実行中または開始に失敗しました",
        "nav_dialog_title": "🧭 ナビゲーションタスク設定",
        "open_nav_dialog": "➕ ナビゲーションタスクダイアログを開く",

        "start_coord_title": "🟢 初期座標点を入力",
        "start_x": "📍 開始点 X",
        "start_y": "📍 開始点 Y",
        "start_yaw": "🧭 開始角度（Yaw）",
        "set_start_button": "✅ 開始点を設定",
        "start_set_success": "✅ 開始点が設定され送信されました：X={}、Y={}、開始角度={:.2f}",
        "ros_node_warning": "⚠️ ROSノードが初期化されていません、初期位置を送信できません",

        "goal_coord_title": "🎯 目標座標点を入力",
        "goal_group": "第{}グループ",
        "goal_x": "🎯 X {}",
        "goal_y": "🎯 Y {}",
        "goal_yaw": "🧭 Yaw {}",
        "add_goal_button": "➕ 目標点を追加",
        "send_nav_button": "✅ タスクを送信",
        "ros_node_error": "❌ ROSノードが初期化されていません",
        "nav_task_success": "📤 目標点が送信されナビゲーションが開始されました",

        "control_panel": "⚙️ コントロールパネル",
        "nav_step1_title": "🟦 ステップ 1：Rviz2 を起動",
        "nav_step1_button": "Rviz2 を起動",
        "nav_step1_loading": "Rviz2 を起動しています。しばらくお待ちください...",
        "nav_step1_success": "✅ Rviz2 が起動しました！",
        "nav_step1_error": "❌ 起動に失敗しました：{error}",
        "nav_step2_button": "セマンティックナビゲーションノードを起動",
        "nav_step2_title": "🟩 セマンティックナビゲーションノードを起動",
        "nav_step2_loading": "セマンティックナビゲーションノードを起動しています。しばらくお待ちください...",
        "nav_step2_success": "✅ Granite + BLIP + YOLO モデルの読み込みが完了しました",
        "nav_step2_warning": "⚠️ モデルを起動中です。数秒後にタスクを送信するか、ログ状態を確認してください。",
        "nav_step2_error": "❌ 起動に失敗しました：{error}",
        "nav_step3_title": "🟩 ステップ 3：初期座標を設定",
        "nav_step3_button": "初期座標を設定",
        "nav_step3_loading": "初期姿勢 (0, 0, 0) を Rviz2 に送信しています...",
        "nav_step3_success": "📍 初期姿勢 (0, 0, 0) の送信に成功しました",
        "nav_step3_error": "❌ 初期姿勢の送信に失敗しました：{error}",
        "nav_step4_title": "🟨 ステップ 4：モデルを選択",
        "nav_step4_select": "モデルタイプを選択：",
        "nav_step4_success": "✅ 選択されたモデル：{model}",
        "nav_step5_title": "🟦 ステップ 5：タスクを入力",
        "nav_step5_placeholder": "タスクの内容を入力してください…",
        "nav_step5_button": "タスクを送信",
        "nav_step5_success": "🧠 タスクが送信されました：{task}",
        "nav_step5_warning": "⚠️ まずタスク内容を入力してください！",
        "nav_step5_error": "❌ 送信に失敗しました：{error}",
        "nav_step6_title": "🟥 ステップ 6：ナビゲーションを終了し、Rviz2 を閉じる",
        "nav_step6_button": "ナビゲーションを終了して Rviz2 を閉じる",
        "nav_step6_loading": "Rviz2 とセマンティックナビゲーションノードを終了しています...",
        "nav_step6_success": "🛑 stop_navigation.sh を正常に実行し、Rviz2 を終了しました！",
        "nav_step6_warning": "⚠️ stop_navigation.sh の実行が完了しましたが、警告が発生した可能性があります：\n{result.stderr}",
        "nav_step6_error": "❌ 終了に失敗しました：{e}",

        "nav_task_order": "📜 タスク実行順序",
        "model_response": "✨ モデルの応答結果",


        "seg_title": "🧠 セマンティックセグメンテーション（リアルタイム）",
        "semantic_caption": "セマンティックセグメンテーション結果",
        "waiting_seg": "⏳ セマンティック画像を待機中...",
        "ros_not_ready": "⚠️ ROSノードが初期化されておらず、セマンティック画像を表示できません",

        "nav_status_title": "🚗 3. ロボットナビゲーション状態概要",
        "total_goals": "📋 **総目標数：** {}個",
        "current_goal_ready": "📍 **現在の目標：** 🚦 開始準備完了",
        "current_goal_progress": "📍 **現在の目標：** 🎯 第{} / {}個",
        "current_goal_return": "📍 **現在の目標：** 🏠 開始点に戻る",
        "nav_status_label": "🧱 ナビゲーション状態：",
        "yolo_title": "🧠 YOLO検出結果",
        "yolo_no_detection": "📷 まだ物体が検出されていません",

        "status_running": "実行中",
        "status_avoiding": "障害物回避中",
        "status_paused": "一時停止中",

        "end_task_title": "🔚 4. タスク終了",
        "close_nav_button": "🛑 ナビゲーション終了",
        "close_nav_success": "✅ ナビゲーションとYOLOが正常に終了しました",
        "no_output": "✅ 標準出力なし",
        "error_output": "❌ エラー出力:\n{}\n{}",
        "execute_failed": "❌ 実行失敗：{}",

        "camera_env_init": "環境初期化",
        "camera_env_info": "以下のコマンドを入力してください",
        "camera_input_command": "Linuxコマンドを入力",
        "camera_execute": "実行",
        "camera_command_success": "コマンドがバックグラウンドで実行されています、PID：",
        "camera_error": "エラーが発生しました：",
        "camera_input_python": "Pythonコマンドを入力",
        "camera_python_success": "Pythonコマンドがバックグラウンドで実行されています、PID：",
        "camera_view_title": "カメラ画面を表示",
        "camera_view_caption": "下のボタンをクリックしてカメラをオン/オフし、リアルタイム画面を表示します。",
        "camera_start": "カメラを開始",
        "camera_stop": "カメラを停止",
        "camera_script_closed": "open_camera.sh が強制終了されました",
        "camera_script_close_error": "open_camera.sh の終了時にエラーが発生しました：",
        "camera_python_closed": "testCameraWeb.py が強制終了されました",
        "camera_python_close_error": "testCameraWeb.py の終了時にエラーが発生しました：",
        "camera_status": "ステータス：",
        "camera_status_on": "開始済み",
        "camera_status_off": "停止済み",
        "camera_image_caption": "ロボットカメラ",
        "camera_no_image": "カメラ画像がまだ検出されていません",
        "camera_control_title": "キーボードでロボットを制御",
        "camera_control_caption": "下のボタンを使ってロボットの前進/後退/左回転/右回転を制御します",
        "camera_send_command": "コマンド送信",
        "camera_speed_setting": "速度設定",
        "camera_linear_speed": "線速度（前後）",
        "camera_angular_speed": "角速度（回転）",
        "camera_keyboard_control": "キーボード方向制御",
        "camera_forward": "W（前進）",
        "camera_left": "A（左回転）",
        "camera_backward": "S（後退）",
        "camera_right": "D（右回転）",
        "camera_emergency_stop": "緊急停止",
        "camera_stop_robot": "ロボットを停止",
        "camera_stop_sent": "停止コマンドを送信しました",

        "susi_status_title": "マシン状態確認（SUSI Node）",
        "susi_status_info": "ターミナルを開いて、以下のコマンドを順番に入力し、ROS2とSUSI環境を初期化してください：",
        "susi_input_command": "Linuxコマンドを入力",
        "susi_start": "SUSI起動",
        "susi_stop": "SUSI終了",
        "susi_starting": "SUSI起動中",
        "susi_exec_error": "実行時にエラーが発生しました：",
        "susi_input_warning": "完全なコマンドを入力してから実行してください",
        "susi_stopped": "SUSIが正常に終了しました",
        "susi_stop_error": "終了時にエラーが発生しました：",
        "susi_not_running": "SUSIがまだ起動していないか、すでに終了しています",

        "susi_monitor_title": "SUSI システムモニター",
        "susi_manual_refresh": "手動更新",
        "susi_auto_refresh": "自動更新",
        "susi_data_error": "データエラー：",
        "susi_missing_time": "system_time フィールドがありません！",
        "susi_data_time": "データ時刻：",
        "susi_outdated": "データが古い",
        "susi_latest": "最新のデータ",
        "susi_monitor_items": "監視項目：",
        "susi_data_outdated_warning": "⚠️ 最新のデータ取得には SUSI コマンドを実行してください",
        "susi_next_refresh": "次の更新：",
        "susi_seconds": "秒後",
        "desc_3v": "（BIOS / コントローラー使用）",
        "desc_5v": "（USB / ハードディスク / 制御回路）",
        "desc_12v": "（ファン / モーター / グラフィックスカード）",
        "desc_cmos": "（マザーボードバッテリー）",
        "voltage_monitor_title": "🔋 電圧モニター",
        "temperature_monitor_title": "🌡️ 温度モニター",
        "temperature_label": "温度",
        "fan_monitor_title": "🌀 ファンモニター",
        "current_monitor_title": "⚡ 電流モニター",
        "disk_monitor_title": "📀 ディスク情報",
        "total_disk_label": "📀 **総ディスク容量**",
        "susi_time_format_error": "データ時間の解析に失敗しました",
        "fan_label": "ファン",
        "cpu_fan_stopped": "CPUファン停止 - 直ちに確認してください！",
        "fan_not_running": "ファンが動作していません",
        "debug_checkbox_label": "デバッグ情報を表示",
        "debug_current_time": "現在時刻: {time:.1f}",
        "debug_last_update": "前回更新: {time:.1f}",
        "debug_time_diff": "時間差: {seconds:.1f}秒",
        "debug_remaining_time": "残り時間: {seconds:.1f}秒",
        "debug_progress": "進捗: {progress:.2%}",
        "debug_data_timestamp": "データタイムスタンプ: {timestamp:.1f}",
        "debug_data_update_diff": "データと更新時間差: {diff:.1f}秒",

        "analysis_title": "🧠 データ分析と可視化",
        "analysis_yolo_caption": "📎 YOLO検出物体の出現回数統計",
        "analysis_yolo_time_expired": "⚠️ 最後の任務から10分以上経過しています（保存時間：{time}）、グラフを表示しません。",
        "analysis_yolo_no_time": "⚠️ 保存時間が見つかりません。データ形式を確認してください。",
        "analysis_yolo_error": "❌ YOLO検出ログの読み込みに失敗しました",
        "analysis_path_caption": "📍 実際の経路と計画経路",
        "analysis_path_no_csv": "⚠️ 経路CSVファイルが見つかりません。ロボットが経路を保存したか確認してください。",
        "analysis_path_empty": "⚠️ 経路データが空です。表示可能な経路がありません。",
        "analysis_path_title": "ロボットナビゲーション経路",
        "analysis_object_caption": "🔥 検出物体の分布図",
        "analysis_object_no_data": "⚠️ 有効な検出データがありません",
        "analysis_object_title": "物体位置の分布図",
        "analysis_time_error": "YOLO検出データの有効期限が切れています",
        "analysis_object_time_expired": "⚠️ オブジェクト検出データが期限切れです（最終更新：{time}）",
        "analysis_path_time_expired": "⚠️ パスデータが期限切れです（最終更新：{time}）",
    },
    "한국어": {
        "power_title": "⏹️ 전원 끄기",
        "power_caption": "아래 버튼을 클릭하면 기기가 종료됩니다. 데이터가 모두 저장되었는지 확인하세요.",
        "power_button": "🔌 전원 끄기",
        "power_warning": "⚠️ 종료 중...",
        "power_error": "⚠️ 종료 명령 실행 중 오류 발생：",

        "reboot_title": "🔁 재시작",
        "reboot_caption": "이 작업을 실행하면 시스템이 즉시 재시작됩니다. 모든 데이터를 저장했는지 확인하세요.",
        "reboot_button": "🔁 지금 재시작",
        "reboot_warning": "⚠️ 시스템을 재시작합니다. 작업 내용을 저장해 주세요.",
        "reboot_error": "❌ 재시작 실패：",

        "language_title": "🌐 언어 설정",
        "language_caption": "인터페이스 언어를 선택하세요. 즉시 적용됩니다.",
        "language_radio": "언어 선택：",
        "language_success": "✅ 현재 언어：",

        "theme_title": "🎨 테마 색상 설정",
        "theme_caption": "시스템 테마 색상을 선택하세요. 전체 스타일에 영향을 줍니다.",
        "theme_radio": "테마 선택：",
        "theme_success": "🎨 현재 테마：",

        "contact_title": "📞 긴급 연락처",
        "contact_caption": "로봇이 고장나거나 응답하지 않거나 안전 문제가 발생한 경우 즉시 아래 담당자에게 연락하세요.",

        "logout_title": "🔓 로그아웃",
        "logout_caption": "아래 버튼을 클릭하면 로그아웃하고 메인 화면으로 돌아갑니다.",
        "logout_button": "🚪 로그아웃",
        "logout_success": "👋 성공적으로 로그아웃되었습니다!",

        "sidebar_title": "🤖 AMR 제어 메뉴",
        "welcome_user": "👋 환영합니다：",
        "honorific": "님",
        "sidebar_pages": ["🏠 홈", "📡 라이다 실행", "📍 내비게이션 작업", "📷 카메라 화면", "📊 로봇 상태", "🗂️ 작업 로그", "⚙️ 시스템 설정"],
        "nav_modes": ["📍 좌표 내비게이션", "💬 의미 내비게이션"],
        "login": "👤 로그인",
        "register": "📝 회원가입",
        "login_modal_title": "🔐 사용자 로그인",
        "register_modal_title": "🆕 계정 생성",
        "login_success": "✅ 다시 오신 걸 환영합니다!",
        "login_error": "❌ 아이디 또는 비밀번호가 올바르지 않습니다.",
        "register_success": "✅ 회원가입 성공!",
        "login_account": "아이디",
        "login_password": "비밀번호",
        "register_account": "아이디 설정",
        "register_password": "비밀번호 설정",
        "register_password2": "비밀번호 다시 입력",
        "login_button": "로그인",
        "register_button": "가입",
        "error_login_required": "⚠️ 먼저 로그인해야 합니다. 메인 페이지로 돌아갑니다.",

        "hardware_title": "🧠 하드웨어 플랫폼 정보 （ Advantech ARK-3533）",
        "hardware_spec_item": "항목",
        "hardware_spec_description": "사양 설명",
        "hardware_cpu": "🖥️ 프로세서 (CPU)",
        "hardware_cpu_desc": "12세대 Intel Core i9 / i7 / i5 / i3（Alder Lake）",
        "hardware_gpu": "🎮 GPU 확장",
        "hardware_gpu_desc": "PCIe x16 외부 GPU 확장 지원（NVIDIA 등）",
        "hardware_ram": "🧠 메모리 (RAM)",
        "hardware_ram_desc": "DDR5 지원, 최대 64GB（SODIMM ×2）",
        "hardware_storage": "💾 저장소",
        "hardware_storage_desc": "M.2 NVMe ×1、SATA SSD/HDD ×2",
        "hardware_network": "🌐 네트워크",
        "hardware_network_desc": "2× GbE LAN, Wi-Fi / 5G / LTE 지원",
        "hardware_io": "🔌 I/O",
        "hardware_io_desc": "USB 3.2, HDMI / DP, COM, CANBus",
        "hardware_temp": "🌡️ 작동 온도",
        "hardware_temp_desc": "-20°C ~ 60°C, 팬리스 설계",
        "hardware_expansion": "🧩 확장성",
        "hardware_expansion_desc": "M.2, mini PCIe, SIM 카드 슬롯",
        "hardware_spec_local": "로컬 정보",
        "hardware_expansion_local": "M.2 ×2 (NVMe 포함), Mini PCIe ×2, PCIe 다수 슬롯, SATA, SIM 카드 슬롯",
        "chatbot_title": "💻 AMR 도우미 ChatBot",
        "chatbot_input": "💬 무엇을 물어보시겠습니까？",
        "chatbot_clear": "🧹 대화 지우기",
        "chatbot_error": "⚠️ 오류가 발생했습니다：",
        "chatbot_system_prompt": "당신은 AMR 도우미로, 로봇 내비게이션, 장애물 회피, 상태 보고 등의 작업을 잘합니다.",

        "radar_title_1": "📡 1. 라이다 시작",
        "radar_info_1": "터미널을 열고 다음 두 줄의 명령어를 입력하세요：",
        "radar_input_command": "Linux 명령어 입력:",
        "radar_execute": "🚀 실행",
        "radar_success": "（✅ 라이다가 성공적으로 시작되었습니다（백그라운드에서 실행 중））",
        "radar_input_warning": "명령어를 입력하세요",
        "radar_title_2": "🖼️ 2. Rviz2 라이다 맵 표시 영역",
        "radar_show_button": "🛰️ 라이다 이미지 표시",
        "radar_show_success": "✅ RViz에서 지속적인 스크린샷을 시작했습니다...",
        "radar_stop_button": "❌ 라이다 이미지 업데이트 중지",
        "radar_stop_success": "🛑 RViz 업데이트를 중지했습니다",
        "radar_image_caption": "📡 라이다 이미지 실시간 표시 중",
        "radar_no_image": "⚠️ 아직 최신 라이다 이미지가 감지되지 않았습니다",
        "radar_title_3": "💾 3. 맵 저장 / 라이다 종료",
        "radar_info_3": "다음 두 줄의 명령어를 입력하세요",
        "radar_no_output": "（출력 없음）",

        "env_init_title": "🔧 1. 환경 초기화",
        "env_init_info": "다음 명령어를 입력하세요",
        "input_command": "Linux 명령어 입력:",
        "execute_button": "🚀 실행",
        "input_warning": "⚠️ 명령어를 입력하세요!",
        "execute_success": "✅ 명령어가 백그라운드에서 실행되었습니다. PID: {}",
        "execute_error": "❌ 실행 중 오류가 발생했습니다: {}",

        "nav_task_title": "🧭 2. 내비게이션 작업 설정",
        "start_nav_node": "🤖 내비게이션 노드 시작",
        "nav_node_success": "✅ 내비게이션 노드가 시작되었습니다",
        "nav_node_info": "ℹ️ 내비게이션 노드가 이미 실행 중이거나 시작에 실패했습니다",
        "nav_dialog_title": "🧭 내비게이션 작업 설정",
        "open_nav_dialog": "➕ 내비게이션 작업 다이얼로그 열기",

        "start_coord_title": "🟢 시작 좌표 입력",
        "start_x": "📍 시작점 X",
        "start_y": "📍 시작점 Y",
        "start_yaw": "🧭 시작 각도(Yaw)",
        "set_start_button": "✅ 시작점 설정",
        "start_set_success": "✅ 시작점이 설정되어 전송되었습니다: X={} , Y={} , 시작 각도={:.2f}",
        "ros_node_warning": "⚠️ ROS 노드가 초기화되지 않았습니다. 시작 위치를 전송할 수 없습니다",

        "goal_coord_title": "🎯 목표 좌표 입력",
        "goal_group": "{}번째 그룹",
        "goal_x": "🎯 X {}",
        "goal_y": "🎯 Y {}",
        "goal_yaw": "🧭 Yaw {}",
        "add_goal_button": "➕ 목표점 추가",
        "send_nav_button": "✅ 작업 전송",
        "ros_node_error": "❌ ROS 노드가 초기화되지 않았습니다",
        "nav_task_success": "📤 목표점이 전송되어 내비게이션이 시작되었습니다",

        "control_panel": "⚙️ 제어 패널",
        "nav_step1_title": "🟦 단계 1: Rviz2 실행",
        "nav_step1_button": "Rviz2 실행",
        "nav_step1_loading": "Rviz2를 실행 중입니다. 잠시만 기다려 주세요...",
        "nav_step1_success": "✅ Rviz2가 실행되었습니다!",
        "nav_step1_error": "❌ 실행에 실패했습니다: {error}",
        "nav_step2_button": "시맨틱 내비게이션 노드 실행",
        "nav_step2_title": "🟩 시맨틱 내비게이션 노드 실행",
        "nav_step2_loading": "시맨틱 내비게이션 노드를 실행 중입니다. 잠시만 기다려 주세요...",
        "nav_step2_success": "✅ Granite + BLIP + YOLO 모델이 로드되었습니다",
        "nav_step2_warning": "⚠️ 모델이 시작 중입니다. 몇 초 후에 작업을 전송하거나 로그 상태를 확인하세요.",
        "nav_step2_error": "❌ 실행에 실패했습니다: {error}",
        "nav_step3_title": "🟩 단계 3: 초기 좌표 설정",
        "nav_step3_button": "초기 좌표 설정",
        "nav_step3_loading": "초기 자세 (0, 0, 0)을(를) Rviz2로 전송 중입니다...",
        "nav_step3_success": "📍 초기 자세 (0, 0, 0) 전송에 성공했습니다",
        "nav_step3_error": "❌ 초기 자세 전송에 실패했습니다: {error}",
        "nav_step4_title": "🟨 단계 4: 모델 선택",
        "nav_step4_select": "모델 유형을 선택:",
        "nav_step4_success": "✅ 선택된 모델: {model}",
        "nav_step5_title": "🟦 단계 5: 작업 입력",
        "nav_step5_placeholder": "작업 설명을 입력하세요…",
        "nav_step5_button": "작업 보내기",
        "nav_step5_success": "🧠 작업이 전송되었습니다: {task}",
        "nav_step5_warning": "⚠️ 먼저 작업 내용을 입력하세요!",
        "nav_step5_error": "❌ 전송에 실패했습니다: {error}",
        "nav_step6_title": "🟥 단계 6: 내비게이션 종료 및 Rviz2 닫기",
        "nav_step6_button": "내비게이션을 종료하고 Rviz2 닫기",
        "nav_step6_loading": "Rviz2와 시맨틱 내비게이션 노드를 종료 중입니다...",
        "nav_step6_success": "🛑 stop_navigation.sh이(가) 성공적으로 실행되어 Rviz2가 종료되었습니다!",
        "nav_step6_warning": "⚠️ stop_navigation.sh 실행이 완료되었지만 경고가 발생했을 수 있습니다:\n{result.stderr}",
        "nav_step6_error": "❌ 종료에 실패했습니다: {e}",

        "nav_task_order": "📜 작업 실행 순서",
        "model_response": "✨ 모델 응답 결과",


        "seg_title": "🧠 실시간 시맨틱 분할 화면",
        "semantic_caption": "시맨틱 분할 결과",
        "waiting_seg": "⏳ 시맨틱 분할 영상을 기다리는 중...",
        "ros_not_ready": "⚠️ ROS 노드가 초기화되지 않아 시맨틱 분할 이미지를 표시할 수 없습니다",


        "nav_status_title": "🚗 3. 로봇 내비게이션 상태 요약",
        "total_goals": "📋 **총 목표 수:** {}개",
        "current_goal_ready": "📍 **현재 목표:** 🚦 시작 준비 완료",
        "current_goal_progress": "📍 **현재 목표:** 🎯 제 {} / {} 개",
        "current_goal_return": "📍 **현재 목표:** 🏠 시작점으로 복귀",
        "nav_status_label": "🧱 내비게이션 상태:",
        "yolo_title": "🧠 YOLO 감지 결과",
        "yolo_no_detection": "📷 아직 객체가 감지되지 않았습니다",

        "status_running": "실행 중",
        "status_avoiding": "장애물 회피 중",
        "status_paused": "일시 중지됨",

        "end_task_title": "🔚 4. 작업 종료",
        "close_nav_button": "🛑 내비게이션 종료",
        "close_nav_success": "✅ 내비게이션과 YOLO가 정상적으로 종료되었습니다",
        "no_output": "✅ 출력 없음",
        "error_output": "❌ 오류 출력:\n{}\n{}",
        "execute_failed": "❌ 실행 실패: {}",

        "camera_env_init": "환경 초기화",
        "camera_env_info": "다음 명령을 입력하세요",
        "camera_input_command": "Linux 명령 입력",
        "camera_execute": "실행",
        "camera_command_success": "명령이 백그라운드에서 실행 중입니다, PID：",
        "camera_error": "오류가 발생했습니다：",
        "camera_input_python": "Python 명령 입력",
        "camera_python_success": "Python 명령이 백그라운드에서 실행 중입니다, PID：",
        "camera_view_title": "카메라 화면 보기",
        "camera_view_caption": "버튼을 클릭하여 카메라를 켜거나 끄고 실시간 화면을 표시합니다.",
        "camera_start": "카메라 시작",
        "camera_stop": "카메라 중지",
        "camera_script_closed": "open_camera.sh가 강제 종료되었습니다",
        "camera_script_close_error": "open_camera.sh 종료 시 오류가 발생했습니다：",
        "camera_python_closed": "testCameraWeb.py가 강제 종료되었습니다",
        "camera_python_close_error": "testCameraWeb.py 종료 시 오류가 발생했습니다：",
        "camera_status": "상태：",
        "camera_status_on": "켜짐",
        "camera_status_off": "꺼짐",
        "camera_image_caption": "로봇 카메라",
        "camera_no_image": "카메라 이미지가 아직 감지되지 않았습니다",
        "camera_control_title": "키보드로 로봇 제어",
        "camera_control_caption": "아래 버튼을 사용하여 로봇의 전진/후진/좌회전/우회전을 제어합니다",
        "camera_send_command": "명령 전송",
        "camera_speed_setting": "속도 설정",
        "camera_linear_speed": "선형 속도（전후）",
        "camera_angular_speed": "각속도（회전）",
        "camera_keyboard_control": "키보드 방향 제어",
        "camera_forward": "W（전진）",
        "camera_left": "A（좌회전）",
        "camera_backward": "S（후진）",
        "camera_right": "D（우회전）",
        "camera_emergency_stop": "긴급 정지",
        "camera_stop_robot": "로봇 정지",
        "camera_stop_sent": "정지 명령을 전송했습니다",

        "susi_status_title": "기계 상태 확인（SUSI Node）",
        "susi_status_info": "터미널을 열고 다음 명령을 순서대로 입력하여 ROS2와 SUSI 환경을 초기화하세요：",
        "susi_input_command": "Linux 명령 입력",
        "susi_start": "SUSI 시작",
        "susi_stop": "SUSI 종료",
        "susi_starting": "SUSI 시작 중",
        "susi_exec_error": "실행 시 오류가 발생했습니다：",
        "susi_input_warning": "완전한 명령을 입력한 후 실행하세요",
        "susi_stopped": "SUSI가 성공적으로 종료되었습니다",
        "susi_stop_error": "종료 시 오류가 발생했습니다：",
        "susi_not_running": "SUSI가 아직 시작되지 않았거나 이미 종료되었습니다",

        "susi_monitor_title": "SUSI 시스템 모니터",
        "susi_manual_refresh": "수동 새로고침",
        "susi_auto_refresh": "자동 새로고침",
        "susi_data_error": "데이터 오류: ",
        "susi_missing_time": "system_time 필드가 없습니다!",
        "susi_data_time": "데이터 시간:",
        "susi_outdated": "데이터가 오래되었습니다",
        "susi_latest": "최신 데이터",
        "susi_monitor_items": "모니터링 항목:",
        "susi_data_outdated_warning": "⚠️ 최신 데이터를 가져오려면 susi 명령을 실행하세요",
        "susi_next_refresh": "다음 새로고침:",
        "susi_seconds": "초 후",
        "desc_3v": "(BIOS / 컨트롤러용)",
        "desc_5v": "(USB / 하드디스크 / 제어 회로)",
        "desc_12v": "(팬 / 모터 / 그래픽 카드)",
        "desc_cmos": "(메인보드 배터리)",
        "voltage_monitor_title": "🔋 전압 모니터",
        "temperature_monitor_title": "🌡️ 온도 모니터",
        "temperature_label": "온도",
        "fan_monitor_title": "🌀 팬 모니터",
        "current_monitor_title": "⚡ 전류 모니터",
        "disk_monitor_title": "📀 디스크 정보",
        "total_disk_label": "📀 **총 디스크 공간**",
        "susi_time_format_error": "데이터 시간 형식을 해석하지 못했습니다: ",
        "fan_label": "팬",
        "cpu_fan_stopped": "CPU 팬 정지 - 즉시 확인하세요!",
        "fan_not_running": "팬이 작동하지 않음",
        "debug_checkbox_label": "디버그 정보 표시",
        "debug_current_time": "현재 시간: {time:.1f}",
        "debug_last_update": "마지막 업데이트: {time:.1f}",
        "debug_time_diff": "시간차: {seconds:.1f}초",
        "debug_remaining_time": "남은 시간: {seconds:.1f}초",
        "debug_progress": "진행률: {progress:.2%}",
        "debug_data_timestamp": "데이터 타임스탬프: {timestamp:.1f}",
        "debug_data_update_diff": "데이터와 업데이트 시간차: {diff:.1f}초",

        "analysis_title": "🧠 데이터 분석 및 시각화",
        "analysis_yolo_caption": "📎 YOLO 감지 객체 출현 횟수 통계",
        "analysis_yolo_time_expired": "⚠️ 마지막 작업이 10분 이상 경과했습니다 (저장 시간: {time}) — 차트를 표시하지 않습니다.",
        "analysis_yolo_no_time": "⚠️ 저장 시간이 없습니다. 로그 형식을 확인해 주세요.",
        "analysis_yolo_error": "❌ YOLO 감지 로그를 읽는 중 오류 발생",
        "analysis_path_caption": "📍 실제 경로 및 계획 경로",
        "analysis_path_no_csv": "⚠️ 경로 CSV 파일이 없습니다. 로봇이 경로를 저장했는지 확인하세요.",
        "analysis_path_empty": "⚠️ 경로 데이터가 비어 있습니다. 표시할 경로가 없습니다.",
        "analysis_path_title": "로봇 이동 경로",
        "analysis_object_caption": "🔥 객체 감지 분포도",
        "analysis_object_no_data": "⚠️ 유효한 감지 데이터가 없습니다",
        "analysis_object_title": "객체 위치 분포도",
        "analysis_time_error": "YOLO 감지 데이터가 만료되었습니다",
        "analysis_object_time_expired": "⚠️ 객체 감지 데이터가 만료되었습니다 (마지막 업데이트: {time})",
        "analysis_path_time_expired": "⚠️ 경로 데이터가 만료되었습니다 (마지막 업데이트: {time})",
    },
    
    "English": {
        "power_title": "⏹️ Power Off",
        "power_caption": "Click the button below to shut down the machine. Make sure all data is saved!",
        "power_button": "🔌 Shut Down",
        "power_warning": "⚠️ Preparing to shut down...",
        "power_error": "⚠️ An error occurred while executing shutdown:",

        "reboot_title": "🔁 Reboot System",
        "reboot_caption": "This operation will immediately restart the system. Please make sure all data is saved.",
        "reboot_button": "🔁 Reboot Now",
        "reboot_warning": "⚠️ System will reboot shortly. Please save your work.",
        "reboot_error": "❌ Reboot failed:",

        "language_title": "🌐 Language Settings",
        "language_caption": "Please select your preferred interface language. It will be applied immediately.",
        "language_radio": "Select Language:",
        "language_success": "✅ Current language: ",

        "theme_title": "🎨 Theme Settings",
        "theme_caption": "Choose your preferred system theme. This affects the overall appearance.",
        "theme_radio": "Select Theme:",
        "theme_success": "🎨 Current theme: ",

        "contact_title": "📞 Emergency Contacts",
        "contact_caption": "In case of robot malfunction, loss of control, or safety incident, contact the following person(s) immediately:",

        "logout_title": "🔓 Log Out",
        "logout_caption": "Click the button below to log out and return to the home page.",
        "logout_button": "🚪 Log Out",
        "logout_success": "👋 Successfully logged out!",

        "sidebar_title": "🤖 AMR Control Panel",
        "welcome_user": "👋 Welcome:",
        "honorific": "Mr./Ms.",
        "sidebar_pages": ["🏠 Home", "📡 Start LiDAR", "📍 Navigation Task", "📷 Camera View", "📊 Robot Status", "🗂️ Task Logs", "⚙️ System Settings"],
        "nav_modes": ["📍 Coordinate Navigation", "💬 Language Navigation"],
        "login": "👤 Login",
        "register": "📝 Register",
        "login_modal_title": "🔐 User Login",
        "register_modal_title": "🆕 Create Account",
        "login_success": "✅ Welcome back!",
        "login_error": "❌ Incorrect username or password",
        "register_success": "✅ Registration successful!",
        "login_account": "Username",
        "login_password": "Password",
        "register_account": "Set Username",
        "register_password": "Set Password",
        "register_password2": "Confirm Password",
        "login_button": "Login",
        "register_button": "Register",
        "error_login_required": "⚠️ Please log in to use this feature. Redirected to home page.",

        "hardware_title": "🧠 Hardware Info (Advantech ARK-3533)",
        "hardware_spec_item": "Item",
        "hardware_spec_description": "Specification",
        "hardware_cpu": "🖥️ CPU",
        "hardware_cpu_desc": "12th Gen Intel Core i9 / i7 / i5 / i3 (Alder Lake)",
        "hardware_gpu": "🎮 GPU Expansion",
        "hardware_gpu_desc": "Supports PCIe x16 for external GPU (e.g., NVIDIA)",
        "hardware_ram": "🧠 Memory (RAM)",
        "hardware_ram_desc": "DDR5 up to 64GB (2× SODIMM)",
        "hardware_storage": "💾 Storage",
        "hardware_storage_desc": "M.2 NVMe ×1, SATA SSD/HDD ×2",
        "hardware_network": "🌐 Network",
        "hardware_network_desc": "2× GbE LAN, supports Wi-Fi / 5G / LTE",
        "hardware_io": "🔌 I/O",
        "hardware_io_desc": "USB 3.2, HDMI / DP, COM, CANBus",
        "hardware_temp": "🌡️ Operating Temperature",
        "hardware_temp_desc": "-20°C to 60°C, fanless design",
        "hardware_expansion": "🧩 Expandability",
        "hardware_expansion_desc": "M.2, mini PCIe, SIM card slot",
        "hardware_spec_local": "Local Info",
        "hardware_expansion_local": "M.2 ×2 (including NVMe), Mini PCIe ×2, Multiple PCIe slots, SATA, SIM card slot",

        "chatbot_title": "💻 AMR Assistant ChatBot",
        "chatbot_input": "💬 What would you like to say?",
        "chatbot_clear": "🧹 Clear Chat",
        "chatbot_error": "⚠️ Error:",
        "chatbot_system_prompt": "You are an AMR assistant specialized in navigation, obstacle avoidance, and status reporting.",

        "radar_title_1": "📡 1. Start LiDAR",
        "radar_info_1": "Open the terminal and enter the following two commands:",
        "radar_input_command": "Enter Linux Command:",
        "radar_execute": "🚀 Execute",
        "radar_success": "✅ LiDAR started (running in background)",
        "radar_input_warning": "Please enter a command",
        "radar_title_2": "🖼️ 2. Rviz2 Radar Map Viewer",
        "radar_show_button": "🛰️ Show Radar Map",
        "radar_show_success": "✅ Rviz screenshot capturing started...",
        "radar_stop_button": "❌ Stop Radar Updates",
        "radar_stop_success": "🛑 Rviz screenshot capturing stopped",
        "radar_image_caption": "📡 Real-Time Radar Display",
        "radar_no_image": "⚠️ No radar image detected yet",
        "radar_title_3": "💾 3. Save Map / Stop LiDAR",
        "radar_info_3": "Please enter the following two commands",
        "radar_no_output": "(No Output)",

        "env_init_title": "🔧 1. Environment Initialization",
        "env_init_info": "Enter the following commands",
        "input_command": "Linux Command:",
        "execute_button": "🚀 Execute",
        "input_warning": "⚠️ Please enter a command!",
        "execute_success": "✅ Command is running in the background. PID: {}",
        "execute_error": "❌ Error while executing command: {}",

        "nav_task_title": "🧭 2. Set Navigation Task",
        "start_nav_node": "🤖 Start Navigation Node",
        "nav_node_success": "✅ Navigation node started",
        "nav_node_info": "ℹ️ Navigation node already running or failed to start",
        "nav_dialog_title": "🧭 Navigation Task Settings",
        "open_nav_dialog": "➕ Open Navigation Task Dialog",

        "start_coord_title": "🟢 Enter Start Position",
        "start_x": "📍 Start X",
        "start_y": "📍 Start Y",
        "start_yaw": "🧭 Start Angle (Yaw)",
        "set_start_button": "✅ Set Start",
        "start_set_success": "✅ Start position set and sent: X={}, Y={}, Yaw={:.2f}",
        "ros_node_warning": "⚠️ ROS node is not initialized. Cannot send start position.",

        "goal_coord_title": "🎯 Enter Goal Position",
        "goal_group": "Group {}",
        "goal_x": "🎯 X {}",
        "goal_y": "🎯 Y {}",
        "goal_yaw": "🧭 Yaw {}",
        "add_goal_button": "➕ Add Goal",
        "send_nav_button": "✅ Send Navigation Task",
        "ros_node_error": "❌ ROS node is not initialized",
        "nav_task_success": "📤 Goal points sent. Navigation started.",

        "control_panel": "⚙️ Control Panel",
        "nav_step1_title": "🟦 Step 1: Launch Rviz2",
        "nav_step1_button": "Launch Rviz2",
        "nav_step1_loading": "Launching Rviz2, please wait...",
        "nav_step1_success": "✅ Rviz2 has been launched!",
        "nav_step1_error": "❌ Failed to launch: {error}",
        "nav_step2_button": "Start Semantic Navigation Node",
        "nav_step2_title": "🟩 Start Semantic Navigation Node",
        "nav_step2_loading": "Starting the Semantic Navigation Node. Please wait...",
        "nav_step2_success": "✅ Granite + BLIP + YOLO models loaded successfully",
        "nav_step2_warning": "⚠️ Models are still starting. Please wait a few seconds before sending a task or checking the log status.",
        "nav_step2_error": "❌ Failed to start: {error}",
        "nav_step3_title": "🟩 Step 3: Set Initial Coordinates",
        "nav_step3_button": "Set Initial Coordinates",
        "nav_step3_loading": "Sending initial pose (0, 0, 0) to Rviz2...",
        "nav_step3_success": "📍 Successfully published initial pose (0, 0, 0)",
        "nav_step3_error": "❌ Failed to send initial pose: {error}",
        "nav_step4_title": "🟨 Step 4: Select Model",
        "nav_step4_select": "Select model type:",
        "nav_step4_success": "✅ Selected model: {model}",
        "nav_step5_title": "🟦 Step 5: Enter Task",
        "nav_step5_placeholder": "Please enter task description…",
        "nav_step5_button": "Send Task",
        "nav_step5_success": "🧠 Task has been sent: {task}",
        "nav_step5_warning": "⚠️ Please enter the task content first!",
        "nav_step5_error": "❌ Failed to send: {error}",
        "nav_step6_title": "🟥 Step 6: End Navigation and Close Rviz2",
        "nav_step6_button": "End Navigation and Close Rviz2",
        "nav_step6_loading": "Closing Rviz2 and the semantic navigation node...",
        "nav_step6_success": "🛑 stop_navigation.sh executed successfully and Rviz2 has been closed!",
        "nav_step6_warning": "⚠️ stop_navigation.sh has finished running, but a warning may have occurred:\n{result.stderr}",
        "nav_step6_error": "❌ Failed to close: {e}",

        "nav_task_order": "📜 Task Execution Order",
        "model_response": "✨ Model Response",


        "seg_title": "🧠 Real-time Semantic Segmentation",
        "semantic_caption": "Semantic Segmentation Result",
        "waiting_seg": "⏳ Waiting for segmentation image...",
        "ros_not_ready": "⚠️ ROS node not initialized. Unable to display segmentation image.",


        "nav_status_title": "🚗 3. Navigation Status Overview",
        "total_goals": "📋 **Total Goals:** {}",
        "current_goal_ready": "📍 **Current Goal:** 🚦 Ready to Start",
        "current_goal_progress": "📍 **Current Goal:** 🎯 {}/{}",
        "current_goal_return": "📍 **Current Goal:** 🏠 Returning to Start",
        "nav_status_label": "🧱 Navigation Status:",
        "yolo_title": "🧠 YOLO Detection Results",
        "yolo_no_detection": "📷 No objects detected yet",

        "status_running": "Running",
        "status_avoiding": "Avoiding Obstacles",
        "status_paused": "Paused",

        "end_task_title": "🔚 4. End Task",
        "close_nav_button": "🛑 Stop Navigation",
        "close_nav_success": "✅ Navigation and YOLO stopped successfully",
        "no_output": "✅ No output",
        "error_output": "❌ Error Output:\n{}\n{}",
        "execute_failed": "❌ Execution failed: {}",

        "camera_env_init": "Environment Initialization",
        "camera_env_info": "Enter the following commands",
        "camera_input_command": "Linux Command",
        "camera_execute": "Execute",
        "camera_command_success": "Command running in background. PID: ",
        "camera_error": "Error occurred:",
        "camera_input_python": "Enter Python Command",
        "camera_python_success": "Python command running in background. PID: ",
        "camera_view_title": "Camera View",
        "camera_view_caption": "Click to open or close the camera and view live feed.",
        "camera_start": "Start Camera",
        "camera_stop": "Stop Camera",
        "camera_script_closed": "open_camera.sh forcibly closed",
        "camera_script_close_error": "Error closing open_camera.sh:",
        "camera_python_closed": "testCameraWeb.py forcibly closed",
        "camera_python_close_error": "Error closing testCameraWeb.py:",
        "camera_status": "Status:",
        "camera_status_on": "On",
        "camera_status_off": "Off",
        "camera_image_caption": "Robot Camera",
        "camera_no_image": "No camera image detected",
        "camera_control_title": "Keyboard Robot Control",
        "camera_control_caption": "Use the buttons below to move the robot forward/back/left/right.",
        "camera_send_command": "Send Command",
        "camera_speed_setting": "Speed Settings",
        "camera_linear_speed": "Linear Speed (Forward/Backward)",
        "camera_angular_speed": "Angular Speed (Rotation)",
        "camera_keyboard_control": "Keyboard Direction Control",
        "camera_forward": "W (Forward)",
        "camera_left": "A (Left)",
        "camera_backward": "S (Backward)",
        "camera_right": "D (Right)",
        "camera_emergency_stop": "Emergency Stop",
        "camera_stop_robot": "Stop Robot",
        "camera_stop_sent": "Stop command sent",

        "susi_status_title": "Robot Status Monitor (SUSI Node)",
        "susi_status_info": "Open terminal and enter the following commands in order to initialize ROS2 and SUSI environment:",
        "susi_input_command": "Linux Command",
        "susi_start": "Start SUSI",
        "susi_stop": "Stop SUSI",
        "susi_starting": "Starting SUSI...",
        "susi_exec_error": "Error while executing:",
        "susi_input_warning": "Please enter a complete command before executing",
        "susi_stopped": "SUSI stopped successfully",
        "susi_stop_error": "Error while stopping SUSI:",
        "susi_not_running": "SUSI not running or already stopped",

        "susi_monitor_title": "SUSI System Monitor",
        "susi_manual_refresh": "Manual Refresh",
        "susi_auto_refresh": "Auto Refresh",
        "susi_data_error": "Data error: ",
        "susi_missing_time": "Missing system_time field!",
        "susi_data_time": "Data Time:",
        "susi_outdated": "Outdated",
        "susi_latest": "Latest",
        "susi_monitor_items": "Monitor Items:",
        "susi_data_outdated_warning": "⚠️ Please run the susi command to get the latest data",
        "susi_next_refresh": "Next refresh:",
        "susi_seconds": "seconds",
        "desc_3v": "(For BIOS / Controllers)",
        "desc_5v": "(USB / HDD / Control Circuit)",
        "desc_12v": "(Fans / Motors / GPU)",
        "desc_cmos": "",
        "voltage_monitor_title": "🔋 Voltage Monitor",
        "temperature_monitor_title": "🌡️ Temperature Monitor",
        "temperature_label": "Temperature",
        "fan_monitor_title": "🌀 Fan Monitor",
        "current_monitor_title": "⚡ Current Monitor",
        "disk_monitor_title": "📀 Disk Info",
        "total_disk_label": "📀 **Total Disk Space**",
        "susi_time_format_error": "Failed to parse data timestamp: ",
        "fan_label": "Fan",
        "cpu_fan_stopped": "CPU Fan Stopped - Check Immediately!",
        "fan_not_running": "Fan Not Running",
        "debug_checkbox_label": "Show Debug Info",
        "debug_current_time": "Current Time: {time:.1f}",
        "debug_last_update": "Last Update: {time:.1f}",
        "debug_time_diff": "Time Difference: {seconds:.1f}s",
        "debug_remaining_time": "Remaining Time: {seconds:.1f}s",
        "debug_progress": "Progress: {progress:.2%}",
        "debug_data_timestamp": "Data Timestamp: {timestamp:.1f}",
        "debug_data_update_diff": "Data vs Update Time Diff: {diff:.1f}s",

        "analysis_title": "🧠 Data Analysis & Visualization",
        "analysis_yolo_caption": "📎 YOLO Detection Count Summary",
        "analysis_yolo_time_expired": "⚠️ Previous task data is older than 10 minutes (saved time: {time}), skipping chart.",
        "analysis_yolo_no_time": "⚠️ Save time not found. Cannot validate data freshness.",
        "analysis_yolo_error": "❌ Failed to load YOLO detection records",
        "analysis_path_caption": "📍 Actual vs Planned Paths",
        "analysis_path_no_csv": "⚠️ Path CSV file not found. Ensure robot saved the path data.",
        "analysis_path_empty": "⚠️ Path data is empty. No route data to show.",
        "analysis_path_title": "Robot Navigation Path",
        "analysis_object_caption": "🔥 Object Detection Distribution",
        "analysis_object_no_data": "⚠️ No valid detection data",
        "analysis_object_title": "Object Position Distribution",
        "analysis_time_error": "YOLO detection data has expired",
        "analysis_object_time_expired": "⚠️ Object detection data has expired (Last update: {time})",
        "analysis_path_time_expired": "⚠️ Path data has expired (Last update: {time})",
    }

}

# 預設語言
if "language" not in st.session_state:
    st.session_state.language = "繁體中文"

lang = st.session_state.language
t = TEXT[lang]  # 目前語系對應的文字表


if "login_modal" not in st.session_state:
    st.session_state.login_modal = False
if "register_modal" not in st.session_state:
    st.session_state.register_modal = False
if "is_logged_in" not in st.session_state:
    st.session_state.is_logged_in = False
if "node" not in st.session_state:
    st.session_state.node = None
if "rviz_pid" not in st.session_state:
    st.session_state.rviz_pid = None
if "semantic_nav_pid" not in st.session_state:
    st.session_state.semantic_nav_pid = None

@st.cache_data(ttl=300)
def load_susi_json():
    with open("/home/amr/Desktop/robot_code/susi/susi_data.json") as f:
        return json.load(f)

def euler2quat(roll, pitch, yaw):
    """歐拉角轉四元數"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return [qw, qx, qy, qz]

class SmartNavNode(Node):
    def __init__(self):
        super().__init__('smart_nav_node')

        self.bridge = CvBridge()
        #self.rgb_subscription = self.create_subscription(RosImage, '/camera/color/image_raw', self.image_callback, 10)
        #self.depth_subscription = self.create_subscription(RosImage, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)

        self.rgb_subscription = None
        self.depth_subscription = None

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # self.bt_log_subscription = self.create_subscription(
        #     String,
        #     '/behavior_tree_log',
        #     self.bt_log_callback,
        #     10
        # )
        
        self.model = YOLO('yolov8l.pt')
        try:
            self.seg_model = YOLO('yolo11l-seg.pt')
            self.get_logger().info("✅ YOLOv11 分割模型载入成功")
        except Exception as e:
            self.get_logger().warn(f"⚠️ YOLOv11 载入失败，使用备用模型: {e}")
            self.seg_model = YOLO('yolov8n-seg.pt')
        self.latest_segmented_image = None
        self.latest_rgb_image = None
        self.segmentation_lock = threading.Lock()
        self.current_detections = []
        self.current_segmentation_results = []
        
        # 预设分割类别颜色
        preset_colors = [
            (255, 69, 0), (138, 43, 226), (220, 20, 60),      # 橙紅、藍紫、深紅
            (255, 0, 0), (0, 255, 0), (0, 0, 255),           # 紅、綠、藍
            (255, 255, 0), (0, 255, 255), (255, 0, 255),     # 黃、青、品紅
            (255, 165, 0), (128, 0, 128), (0, 128, 128),     # 橙、紫、青綠
            (128, 128, 0), (192, 192, 192), (255, 20, 147),  # 橄欖、銀、深粉
            (30, 144, 255), (255, 140, 0), (50, 205, 50),    # 藍、橙紅、萊姆綠
            (255, 99, 71), (75, 0, 130), (255, 215, 0),      # 蕃茄紅、靛青、金色
            (0, 191, 255), (255, 105, 180), (34, 139, 34),   # 深天藍、熱粉、森林綠
        ]
        self.seg_colors = {i: preset_colors[i % len(preset_colors)] for i in range(80)}
        
        self.latest_depth_image = None
        self.avoidance_mode = False
        self.distance_threshold = 1.0
        self.ui_obstacle_threshold = 0.5
        self.safe_counter = 0
        self.safe_threshold = 20
        self.home_position = None
        self.home_orientation = None
        self.home_position_saved = False  # 新增

        self.goal_queue = []
        self.original_goal_count = 0 
        self.returning_home = False
        self.current_goal_pose = None
        self.wait_timer = None
        self.path_history = []
        self.goal_path = []
        self.navigation_resumed = False
        self.is_waiting = False
        self.obstacle_detected_during_this_goal = False
        self.home_position = None 
        self.home_orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        self.is_navigating = False
        self.current_goal_index = 0
        self.total_goals = 0
        self.navigation_active = False
        self.is_avoiding_obstacle = False
        self.keep_speaking = False
        self.speech_thread = None

        self.warning_speech_active = False
        self.warning_speech_thread = None
        self.audio_lock = threading.Lock()
        self.last_warning_time = 0
        self.warning_cooldown = 1.0 
        self.normal_speech_paused = False  
        self.current_audio_process = None

        self.camera_initialized = False
        self.max_log_size = 10 * 1024 * 1024  # 10MB
        self.max_log_entries = 1000


        self.get_logger().info("🟢 智慧導航節點已啟動")
        self.ui_timer = self.create_timer(1.0, self.update_ui_status)
        self.detections_for_ui = []
        self.detection_timer = self.create_timer(1.0, self.save_yolo_detections_to_json)

        self.get_logger().info("⏳ 将在2秒后初始化摄像头...")
        self.camera_init_timer = self.create_timer(2.0, self.delayed_camera_initialization)

    def delayed_camera_initialization(self):
        """延迟初始化摄像头订阅"""
        try:
            self.get_logger().info("🎥 开始初始化摄像头订阅...")
            
            self.rgb_subscription = self.create_subscription(
                RosImage, 
                '/camera/color/image_raw', 
                self.image_callback, 
                10
            )
            
            self.depth_subscription = self.create_subscription(
                RosImage, 
                '/camera/aligned_depth_to_color/image_raw', 
                self.depth_callback, 
                10
            )
            
            self.camera_initialized = True
            
            self.get_logger().info("✅ 摄像头订阅初始化完成")
            
            if self.camera_init_timer is not None:
                self.camera_init_timer.cancel()
                self.camera_init_timer = None
                
        except Exception as e:
            self.get_logger().error(f"❌ 摄像头初始化失败: {e}")
            self.get_logger().info("🔄 5秒后重试摄像头初始化...")
            if self.camera_init_timer is not None:
                self.camera_init_timer.cancel()
            self.camera_init_timer = self.create_timer(5.0, self.delayed_camera_initialization)

    def update_ui_status(self):
        status = "避障中" if self.is_avoiding_obstacle else (
             "進行中" if self.navigation_active else "暫停中")

        ui_data = {
            "total_goals": self.total_goals,
            "current_goal_index": self.current_goal_index,
            "navigation_status": status
        }

        try:
            with open("/home/amr/Desktop/robot_code/ui_status/ui_status.json", "w") as f:
                json.dump(ui_data, f, ensure_ascii=False, indent=2)
            self.get_logger().debug("✅ UI JSON 狀態已更新")
        except Exception as e:
            self.get_logger().warn(f"❗ 寫入 ui_status.json 失敗: {e}")

    def pause_normal_speech(self):
        if self.keep_speaking and not self.normal_speech_paused:
            self.normal_speech_paused = True
            self.get_logger().info("⏸️ 暫停循環語音")
            if self.current_audio_process is not None:
                self.current_audio_process.terminate()
                self.get_logger().infodd ("🛑 強制停止目前播放中的普通語音")
                self.current_audio_process = None

    def resume_normal_speech(self):
        """恢復循環語音"""
        if self.keep_speaking and self.normal_speech_paused:
            self.normal_speech_paused = False
            self.get_logger().info("▶️ 恢復循環語音")

        if self.speech_thread is None or not self.speech_thread.is_alive():
            self.get_logger().info("🔧 语音线程已停止，重新启动")
            self.start_loop_speech()

    def play_warning_speech(self):
        def warning_speech():
            try:
                text_to_speak = "危ないよ、もうちょっと離れて〜"
                output_wav = f"warning_output_{int(time.time() * 1000) % 10000}.wav"

                subprocess.run([
                    "open_jtalk",
                    "-x", "/var/lib/mecab/dic/open-jtalk/naist-jdic",
                    "-m", "/home/amr/Desktop/robot_code/MMDAgent_Example-1.7/Voice/mei/mei_happy.htsvoice",
                    "-r", "0.8",
                    "-fm", "2.0",
                    "-ow", output_wav
                ], input=text_to_speak.encode("utf-8"), timeout=5)

                if os.path.exists(output_wav):
                    with self.audio_lock:
                        subprocess.run(["aplay", output_wav], timeout=5)
                    self.get_logger().info("🚨 播放警告語音：距離過近！")
                    
                    try:
                        os.remove(output_wav)
                    except:
                        pass
                else:
                    self.get_logger().error("❌ 警告语音文件未生成")

            except subprocess.TimeoutExpired:
                self.get_logger().error("❌ 警告语音播放超时")
            except Exception as e:
                self.get_logger().error(f"❌ 警告語音播放錯誤: {e}")
            finally:
                self.warning_speech_active = False
                if self.navigation_active and not self.is_avoiding_obstacle:
                    self.resume_normal_speech()

        if not self.warning_speech_active:
            self.warning_speech_active = True
            self.warning_speech_thread = threading.Thread(target=warning_speech, daemon=True)
            self.warning_speech_thread.start()


    def start_loop_speech(self):
        """开始循环播放语音 - 修复卡顿版本"""
        if self.keep_speaking:
            self.stop_loop_speech()
            time.sleep(0.8)  
        
        self.keep_speaking = True
        self.normal_speech_paused = False
        self.is_avoiding_obstacle = False

        def speech_loop():
            text_to_speak = "まもなく、ロボットが通過いたします。危険ですから、足元にご注意ください〜"
            consecutive_errors = 0  
            max_consecutive_errors = 3  
            
            while self.keep_speaking:
                if self.normal_speech_paused:
                    time.sleep(0.2)
                    consecutive_errors = 0  
                    continue

                try:
                    if not self.keep_speaking or self.normal_speech_paused:
                        break

                    output_wav = f"output_{int(time.time() * 1000) % 10000}.wav" 
                    
                    process_result = subprocess.run([
                        "open_jtalk",
                        "-x", "/var/lib/mecab/dic/open-jtalk/naist-jdic",
                        "-m", "/home/amr/Desktop/robot_code/MMDAgent_Example-1.7/Voice/mei/mei_happy.htsvoice",
                        "-r", "0.8",      
                        "-fm", "2.0",
                        "-ow", output_wav
                    ], input=text_to_speak.encode("utf-8"), timeout=5, capture_output=True)
                    
                    if process_result.returncode != 0:
                        consecutive_errors += 1
                        self.get_logger().error(f"open_jtalk失败，返回码: {process_result.returncode}")
                        if consecutive_errors >= max_consecutive_errors:
                            self.get_logger().error("连续语音生成失败，停止语音播放")
                            break
                        time.sleep(1)
                        continue
                    
                    if not os.path.exists(output_wav):
                        consecutive_errors += 1
                        self.get_logger().error(f"语音文件 {output_wav} 未生成")
                        if consecutive_errors >= max_consecutive_errors:
                            break
                        time.sleep(1)
                        continue

                    if not self.keep_speaking or self.normal_speech_paused:
                        try:
                            os.remove(output_wav)
                        except:
                            pass
                        break

                    with self.audio_lock:
                        if self.keep_speaking and not self.normal_speech_paused:
                            try:
                                self.current_audio_process = subprocess.Popen(
                                    ["aplay", output_wav],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL
                                )
                                
                                start_time = time.time()
                                timeout = 10  
                                
                                while self.current_audio_process.poll() is None:
                                    if not self.keep_speaking or self.normal_speech_paused:
                                        self.get_logger().info("语音播放被中断，正在停止...")
                                        self.current_audio_process.terminate()
                                        try:
                                            self.current_audio_process.wait(timeout=1)
                                        except subprocess.TimeoutExpired:
                                            self.current_audio_process.kill()
                                        break
                                    
                                    if time.time() - start_time > timeout:
                                        self.get_logger().error(f"语音播放超时，强制停止")
                                        self.current_audio_process.terminate()
                                        try:
                                            self.current_audio_process.wait(timeout=1)
                                        except subprocess.TimeoutExpired:
                                            self.current_audio_process.kill()
                                        consecutive_errors += 1
                                        break
                                    
                                    time.sleep(0.1)

                                if self.current_audio_process and self.current_audio_process.poll() == 0:
                                    consecutive_errors = 0
                                    self.get_logger().debug("语音播放完成")
                                
                            except Exception as e:
                                self.get_logger().error(f"播放语音时出错: {e}")
                                consecutive_errors += 1
                            finally:
                                self.current_audio_process = None
                                try:
                                    if os.path.exists(output_wav):
                                        os.remove(output_wav)
                                except Exception as e:
                                    self.get_logger().debug(f"清理语音文件失败: {e}")

                    if not self.keep_speaking or self.normal_speech_paused:
                        break
                    
                    if consecutive_errors >= max_consecutive_errors:
                        self.get_logger().error("连续播放失败次数过多，停止语音播放")
                        break

                    for i in range(10): 
                        if not self.keep_speaking or self.normal_speech_paused:
                            break
                        time.sleep(0.1)

                except subprocess.TimeoutExpired:
                    self.get_logger().error("open_jtalk进程超时")
                    consecutive_errors += 1
                    if consecutive_errors >= max_consecutive_errors:
                        break
                except Exception as e:
                    self.get_logger().error(f"語音播放錯誤: {e}")
                    consecutive_errors += 1
                    if consecutive_errors >= max_consecutive_errors:
                        break
                    time.sleep(1) 

            self.get_logger().info("语音播放线程结束")

        self.speech_thread = threading.Thread(target=speech_loop, daemon=True)
        self.speech_thread.start()
        self.get_logger().info("🔊 開始循環播放語音")

    def stop_loop_speech(self):
        """停止循环播放语音 - 强化版本"""
        self.keep_speaking = False
        self.normal_speech_paused = False 
        
        if self.current_audio_process is not None:
            try:
                self.get_logger().info("正在强制停止当前音频进程...")
                self.current_audio_process.terminate()
                try:
                    self.current_audio_process.wait(timeout=1.0)
                    self.get_logger().info("音频进程已优雅停止")
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("音频进程未能优雅停止，强制杀死")
                    self.current_audio_process.kill()
                    try:
                        self.current_audio_process.wait(timeout=0.5)
                    except subprocess.TimeoutExpired:
                        self.get_logger().error("无法杀死音频进程")
            except Exception as e:
                self.get_logger().error(f"停止音频进程时出错: {e}")
            finally:
                self.current_audio_process = None
        
        try:
            import glob
            wav_files = glob.glob("output_*.wav")
            for wav_file in wav_files:
                try:
                    os.remove(wav_file)
                    self.get_logger().debug(f"清理语音文件: {wav_file}")
                except:
                    pass
        except Exception as e:
            self.get_logger().debug(f"清理语音文件时出错: {e}")
        
        self.get_logger().info("🔇 停止循環播放語音")

    def start_navigation(self):
        if not self.goal_queue:
            self.get_logger().warn("⚠️ 目標列表為空，無法啟動導航")
            return

        if not self.camera_initialized:
            self.get_logger().info("🎥 相機尚未啟動，開始重新初始化...")
            self.delayed_camera_initialization()

        self.start_loop_speech()
        
        self.get_logger().info("🚦 開始處理導航隊列")
        self.process_next_goal()

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if not self.home_position_saved:
            self.home_position = (x, y)
            self.home_orientation = {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
            self.home_position_saved = True
            self.get_logger().info(f"🏠 起始位置已記錄：({x:.3f}, {y:.3f})")
        
        if not self.path_history or math.hypot(x - self.path_history[-1][0], y - self.path_history[-1][1]) > 0.05:
            self.path_history.append((x, y))

    def set_goal_queue(self, goals):
        self.goal_queue = goals.copy()
        self.original_goal_count = len(goals) 
        self.goal_path = []
        self.total_goals = len(goals) + 1  
        self.update_ui_status()
        for i, goal in enumerate(goals):
            x, y, yaw_deg = goal
            self.get_logger().info(f"📋 目標 {i+1}: ({x:.2f}, {y:.2f}) 角度：{yaw_deg:.1f}°")
        self.get_logger().info(f"📋 已設定 {len(goals)} 個導航目標點 + 1個回家點")

    def navigate_to_pose(self, pose_msg):
        self.current_goal_pose = pose_msg
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ 導航伺服器未就緒")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        self.get_logger().info("🚀 發送導航指令中...")
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ 導航目標被拒絕")
            return
        self.get_logger().info("🟢 導航目標已接受，執行中...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """导航结果回调 - 修复版本"""
        self.get_logger().info("📬 收到導航完成 callback")
        result = future.result().result
        self.get_logger().info(f'🌟 導航完成，結果：{result}')
        
        # twist = Twist()
        # twist.linear.x = 0.0
        # twist.angular.z = 0.0
        # self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("🚫 小車已停止")

        self.stop_all_speech()
        
        time.sleep(0.3)
        
        self.play_arrival_speech()

        if self.returning_home:
            self.get_logger().info("🏠 已到達原點")
        else:
            self.get_logger().info("🎯 已到達目標點")

        self.get_logger().info("⏳ 停留 22 秒中...")

        self.save_goal_log(self.current_goal_pose)
        self.navigation_resumed = False
        self.navigation_active = False
        self.is_waiting = True
        self.update_ui_status()

        if self.wait_timer is not None:
            self.wait_timer.cancel()
        self.wait_timer = self.create_timer(22.0, self.wait_completed_callback)

        if not self.returning_home:
            self.goodbye_timer = self.create_timer(20.0, self.play_goodbye_speech)
    
    def play_goodbye_speech(self):
        """播放告別語音 - 在第20秒時播放"""
        def goodbye_speech():
            try:
                text_to_speak = "シェイシェイ～ ザイジエン～"
                output_wav = "goodbye_output.wav"

                subprocess.run([
                    "open_jtalk",
                    "-x", "/var/lib/mecab/dic/open-jtalk/naist-jdic",
                    "-m", "/home/amr/Desktop/robot_code/MMDAgent_Example-1.7/Voice/mei/mei_happy.htsvoice",
                    "-r", "0.9",
                    "-fm", "1.5",
                    "-ow", output_wav
                ], input=text_to_speak.encode("utf-8"), check=True)

                with self.audio_lock:
                    subprocess.run(["aplay", output_wav], check=True)

                self.get_logger().info("👋 播放告別語音：謝謝，再見")

            except Exception as e:
                self.get_logger().error(f"告別語音播放錯誤: {e}")
            finally:
                if hasattr(self, 'goodbye_timer') and self.goodbye_timer is not None:
                    self.goodbye_timer.cancel()
                    self.goodbye_timer = None

        threading.Thread(target=goodbye_speech, daemon=True).start()

    def stop_all_speech(self):
        """停止所有语音播放 - 修复版本"""
        self.keep_speaking = False
        self.normal_speech_paused = False
        
        if self.current_audio_process is not None:
            try:
                self.current_audio_process.terminate()
                try:
                    self.current_audio_process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    self.current_audio_process.kill()
                self.get_logger().info("🛑 强制停止当前播放中的普通语音")
            except Exception as e:
                self.get_logger().error(f"停止普通语音时出错: {e}")
            finally:
                self.current_audio_process = None
        
        self.warning_speech_active = False
        
        self.is_avoiding_obstacle = False
        
        self.get_logger().info("🔇 已停止所有语音播放")

    def play_arrival_speech(self):
        """播放到达语音 - 非阻塞修正版"""
        def arrival_speech():
            try:
                if self.current_audio_process is not None:
                    self.current_audio_process.terminate()
                    self.current_audio_process = None
                
                if self.returning_home:
                    text_to_speak = "終点、スタート地点に到着です。お忘れ物のないようご注意ください。本日はご利用いただき、ありがとうございました。"
                else:
                    text_to_speak = "ご注文の品をお届けしました。どうぞお受け取りください。"

                output_wav = "arrival_output.wav"

                subprocess.run([
                    "open_jtalk",
                    "-x", "/var/lib/mecab/dic/open-jtalk/naist-jdic",
                    "-m", "/home/amr/Desktop/robot_code/MMDAgent_Example-1.7/Voice/mei/mei_happy.htsvoice",
                    "-r", "0.9",
                    "-fm", "0.5",
                    "-ow", output_wav
                ], input=text_to_speak.encode("utf-8"), check=True)

                with self.audio_lock:
                    subprocess.run(["aplay", output_wav], check=True)

            except Exception as e:
                self.get_logger().error(f"到达语音播放错误: {e}")

        threading.Thread(target=arrival_speech, daemon=True).start()


    def save_goal_log(self, pose):
        log_dir = os.path.expanduser("~/Desktop/robot_code/record")
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, "goal_log.csv")

        q = pose.pose.orientation
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        )
        yaw_deg = math.degrees(yaw_rad)

        x = pose.pose.position.x
        y = pose.pose.position.y
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        goal_status = "success"
        wait_time = 22.0
        obstacle_flag = "yes" if self.obstacle_detected_during_this_goal else "no"

        write_header = not os.path.exists(log_path)
        with open(log_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if write_header:
                writer.writerow(["x", "y", "orientation_yaw", "timestamp", "goal_status", "wait_time", "obstacle_encountered"])
            writer.writerow([x, y, yaw_deg, timestamp, goal_status, wait_time, obstacle_flag])

        self.get_logger().info(f"📝 導航紀錄：({x:.2f}, {y:.2f}) | 朝向 {yaw_deg:.1f}° | 遇障：{obstacle_flag}")
        self.obstacle_detected_during_this_goal = False

    def wait_completed_callback(self):
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

        if hasattr(self, 'goodbye_timer') and self.goodbye_timer is not None:
            self.goodbye_timer.cancel()
            self.goodbye_timer = None

        self.is_waiting = False
        self.update_ui_status()
        self.get_logger().info("⌛ 等待時間結束")
        
        if self.returning_home:
            self.plot_path()
            self.append_save_time_to_yolo_log()
            self.get_logger().info("✅ 已回到起點，任務全部完成，等待新目標...")
            
            self.returning_home = False
            self.navigation_active = False
            self.goal_queue = []
            self.current_goal_index = 0
            self.total_goals = 0
            self.navigation_resumed = False
            self.update_ui_status()

        else:
            self.process_next_goal()

    def process_next_goal(self):
        """处理下一个目标 - 修复版本"""
        self.navigation_resumed = False
        
        completed_goals = self.original_goal_count - len(self.goal_queue)
        
        if len(self.goal_queue) == 0 and completed_goals >= self.original_goal_count:
            if self.home_position is None:
                self.get_logger().warn("⚠️ 未記錄起始位置，無法回家")
                self.plot_path()
                self.append_save_time_to_yolo_log()
                self.get_logger().info("✅ 所有目標已完成，等待新任務...")
                
                self.returning_home = False
                self.navigation_active = False
                self.goal_queue = []
                self.current_goal_index = 0
                self.total_goals = 0
                self.navigation_resumed = False
                self.update_ui_status()

                return
                # self.get_logger().warn("⚠️ 未記錄起始位置，無法回家")
                # self.plot_path()
                # self.get_logger().info("✅ 所有目標已完成，結束系統")
                # rclpy.shutdown()
                # cv2.destroyAllWindows()
                # return

            self.stop_all_speech()
            time.sleep(0.5) 
            
            self.start_loop_speech()
            self.get_logger().info("🔊 開始回家，重新開始循環播放語音")
            
            self.returning_home = True
            self.current_goal_index += 1
            self.navigation_active = True
            self.update_ui_status()
            
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = self.home_position[0]
            goal.pose.position.y = self.home_position[1]
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = self.home_orientation['x']
            goal.pose.orientation.y = self.home_orientation['y']
            goal.pose.orientation.z = self.home_orientation['z']
            goal.pose.orientation.w = self.home_orientation['w']
            
            self.get_logger().info(f"🏠 回到起始點：({self.home_position[0]:.2f}, {self.home_position[1]:.2f})")
            self.navigate_to_pose(goal)
            return

        if self.goal_queue:
            next_goal = self.goal_queue.pop(0)
            x, y, yaw_deg = next_goal  
            
            self.goal_path.append((x, y))

            self.current_goal_index += 1
            self.navigation_active = True
            self.update_ui_status()

            self.stop_all_speech()
            time.sleep(0.5) 
            
            self.start_loop_speech()
            self.get_logger().info("🔊 前往下一個目標，重新開始循環播放語音")

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            
            yaw_rad = math.radians(yaw_deg)
            quat = euler2quat(0.0, 0.0, yaw_rad) 
            
            goal.pose.orientation.w = quat[0]  
            goal.pose.orientation.x = quat[1]  
            goal.pose.orientation.y = quat[2]  
            goal.pose.orientation.z = quat[3]  

            self.get_logger().info(f"🎯 前往下一個目標：({x:.2f}, {y:.2f}) 角度：{yaw_deg:.1f}°")
            self.navigate_to_pose(goal)

    def pause_normal_speech(self):
        """暂停普通语音 - 修复版本"""
        if self.keep_speaking and not self.normal_speech_paused:
            self.normal_speech_paused = True
            self.get_logger().info("⏸️ 暫停循環語音")
            
            if self.current_audio_process is not None:
                try:
                    self.current_audio_process.terminate()
                    try:
                        self.current_audio_process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        self.current_audio_process.kill()
                    self.get_logger().info("🛑 強制停止目前播放中的普通語音")
                except Exception as e:
                    self.get_logger().error(f"暂停语音时出错: {e}")
                finally:
                    self.current_audio_process = None

    def draw_segmentation(self, image, results):
        """繪製語義分割結果 - 增強版"""
        if results.masks is None:
            return image
            
        overlay = image.copy()
        masks = results.masks.data.cpu().numpy()
        boxes = results.boxes.data.cpu().numpy()
        
        segmentation_info = []
        
        for mask, box in zip(masks, boxes):
            cls_id = int(box[5])
            confidence = float(box[4])
            
            if confidence < 0.5:  
                continue
                
            mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
            mask_bool = mask_resized > 0.5
            
            color = self.seg_colors.get(cls_id, (255, 255, 255))
            
            overlay[mask_bool] = color
            
            y_coords, x_coords = np.where(mask_bool)
            if len(y_coords) > 0:
                center_y, center_x = int(np.mean(y_coords)), int(np.mean(x_coords))
                area = len(y_coords)
                
                class_name = self.seg_model.names[cls_id]
                label = f"{class_name}: {confidence:.2f}"
                
                (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(overlay, 
                            (center_x - text_width//2 - 5, center_y - text_height - 5),
                            (center_x + text_width//2 + 5, center_y + 5),
                            (0, 0, 0), -1)
                
                cv2.putText(overlay, label, (center_x - text_width//2, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                segmentation_info.append({
                    "class_name": class_name,
                    "confidence": confidence,
                    "center_x": center_x,
                    "center_y": center_y,
                    "area": area,
                    "color": color
                })
        
        self.current_segmentation_results = segmentation_info
    
        return cv2.addWeighted(image, 0.6, overlay, 0.4, 0)


    def image_callback(self, msg):
        if not self.camera_initialized:
            self.get_logger().debug("⏳ 摄像头尚未完全初始化，跳过本次回调")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model(cv_image, conf=0.6)[0]
            current_detections = []
            
            current_pose = self.path_history[-1] if self.path_history else (None, None)
            x_pose, y_pose = current_pose
            
            min_object_distance = float('inf')
            has_close_object = False
            
            for result in results.boxes.data:
                x1, y1, x2, y2, conf, cls = result.cpu().numpy()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                depth = self.get_depth_at_point(cx, cy)
                
                obj = {
                    "物件": self.model.names[int(cls)],
                    "信心分數": round(float(conf), 2),
                    "距離(m)": round(float(depth), 2),
                    "x": round(float(x_pose), 2) if x_pose is not None else None,
                    "y": round(float(y_pose), 2) if y_pose is not None else None
                }
                current_detections.append(obj)
                
                if depth > 0 and depth < 1.0:
                    has_close_object = True
                    min_object_distance = min(min_object_distance, depth)
            
            self.detections_for_ui = current_detections
            
            if self.navigation_active and has_close_object:
                current_time = time.time()
                if (current_time - self.last_warning_time > self.warning_cooldown and 
                    not self.warning_speech_active):
                    self.play_warning_speech()
                    self.last_warning_time = current_time
                    self.get_logger().warn(f"⚠️ YOLO物體距離警告！最近物體距離：{min_object_distance:.2f}m")
                    
                    if not self.is_avoiding_obstacle:
                        self.pause_normal_speech()
                        self.is_avoiding_obstacle = True
            else:
                if self.is_avoiding_obstacle:
                    self.resume_normal_speech()
                    self.is_avoiding_obstacle = False
            
            seg_results = self.seg_model(cv_image, conf=0.5)[0]
            segmented_image = self.draw_segmentation(cv_image, seg_results)
            
            with self.segmentation_lock:
                self.latest_segmented_image = segmented_image
                
        except Exception as e:
            self.get_logger().error(f'影像處理錯誤: {str(e)}')

    def depth_callback(self, msg):
        if not self.camera_initialized:
            self.get_logger().debug("⏳ 摄像头尚未完全初始化，跳过本次深度回调")
            return

        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'深度影像錯誤: {str(e)}')

    def get_depth_at_point(self, x, y, kernel_size=5):
        if self.latest_depth_image is None:
            return -1
        
        h, w = self.latest_depth_image.shape
        x, y = int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))
        hk = kernel_size // 2
        x0, x1 = max(0, x - hk), min(w, x + hk + 1)
        y0, y1 = max(0, y - hk), min(h, y + hk + 1)
        roi = self.latest_depth_image[y0:y1, x0:x1]
        valid = roi[(roi > 0) & (~np.isnan(roi))]
        return np.mean(valid) / 1000.0 if valid.size > 0 else -1

    def stop_camera_subscription(self):
        """停止相機的影像與深度訂閱"""
        if self.rgb_subscription is not None:
            self.destroy_subscription(self.rgb_subscription)
            self.rgb_subscription = None
            self.get_logger().info("🛑 已停止RGB影像訂閱")
        
        if self.depth_subscription is not None:
            self.destroy_subscription(self.depth_subscription)
            self.depth_subscription = None
            self.get_logger().info("🛑 已停止深度影像訂閱")
        
        self.camera_initialized = False


    def save_yolo_detections_to_json(self):
        try:
            detection_data = {
                "YOLO偵測結果": self.detections_for_ui
            }
            with open("/home/amr/Desktop/robot_code/ui_status/yolo_status.json", "w") as f:
                json.dump(detection_data, f, ensure_ascii=False, indent=2)
            self.get_logger().debug("✅ YOLO JSON 狀態已更新")

            log_path = "/home/amr/Desktop/robot_code/ui_status/yolo_full_log.json"

            if os.path.exists(log_path):
                file_size = os.path.getsize(log_path)
                if file_size > 10 * 1024 * 1024:
                    self.get_logger().info("📁 YOLO日志文件过大，重新开始记录")
                    data = []
                else:
                    try:
                        with open(log_path, "r") as f:
                            data = json.load(f)
                        if not isinstance(data, list):
                            data = []
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"❌ JSON文件损坏，重新开始记录: {e}")
                        data = []
                    except Exception as e:
                        self.get_logger().error(f"❌ 读取JSON文件失败，重新开始记录: {e}")
                        data = []
            else:
                data = []

            data.append(detection_data)

            if len(data) > 1000:
                data = data[-1000:]

            with open(log_path, "w") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            self.get_logger().debug("✅ YOLO 偵測結果已追加寫入")
        except Exception as e:
            self.get_logger().warn(f"❗ 寫入 yolo_status.json 失敗: {e}")

    from datetime import datetime

    def append_save_time_to_yolo_log(self):
        try:
            log_path = "/home/amr/Desktop/robot_code/ui_status/yolo_full_log.json"

            if os.path.exists(log_path):
                with open(log_path, "r") as f:
                    data = json.load(f)
                if not isinstance(data, list):
                    data = []
            else:
                data = []

            data.append({
                "保存時間": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            })

            with open(log_path, "w") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            self.get_logger().info("🕒 已在 YOLO 記錄最後追加保存時間")

        except Exception as e:
            self.get_logger().error(f"❗ 寫入保存時間失敗: {e}")

    def plot_path(self):
        if not self.path_history:
            self.get_logger().warn("⚠️ 尚無路徑資料可繪圖")
            return

        actual_x, actual_y = zip(*self.path_history)

        plan_x, plan_y = [], []
        if self.goal_path and len(self.goal_path) > 0:
            plan_x, plan_y = zip(*self.goal_path)

        plt.figure()
        plt.plot(actual_x, actual_y, marker='o', linestyle='-', color='blue', label='Real Route')
        if plan_x and plan_y:
            plt.plot(plan_x, plan_y, marker='x', linestyle='--', color='red', label='Plan Route')

        plt.title("Robot Navigation Path")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.axis('equal')
        plt.legend()

        plot_path = os.path.expanduser("~/Desktop/robot_code/picture_record/path_plot.png")
        plt.savefig(plot_path)
        plt.show()
        plt.close()
        self.get_logger().info(f"📈 小車路徑已儲存於：{plot_path}")

        csv_path = os.path.expanduser("~/Desktop/robot_code/picture_record/path_data_for_streamlit.csv")
        try:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            max_len = max(len(actual_x), len(plan_x) if plan_x else 0)
            rows = []
            for i in range(max_len):
                row = {
                    "Real_X": actual_x[i] if i < len(actual_x) else None,
                    "Real_Y": actual_y[i] if i < len(actual_y) else None,
                    "Plan_X": plan_x[i] if plan_x and i < len(plan_x) else None,
                    "Plan_Y": plan_y[i] if plan_y and i < len(plan_y) else None,
                }
                rows.append(row)
            time_row = {
                "Real_X": "GeneratedTime",
                "Real_Y": current_time,
                "Plan_X": None,
                "Plan_Y": None
            }
            rows.append(time_row)

            with open(csv_path, mode='w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=["Real_X", "Real_Y", "Plan_X", "Plan_Y"])
                writer.writeheader()
                writer.writerows(rows)

            self.get_logger().info(f"📄 小車路徑資料已儲存於：{csv_path}")
        except Exception as e:
            self.get_logger().error(f"❌ 儲存 CSV 失敗：{e}")

def publish_initial_pose(node, x=0.0, y=0.0, yaw_deg=0.0):
    """發布初始位置"""
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    q = euler2quat(0, 0, math.radians(yaw_deg))
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = 'map'
    pose_msg.pose.pose.position.x = x
    pose_msg.pose.pose.position.y = y
    pose_msg.pose.pose.orientation.x = q[1]
    pose_msg.pose.pose.orientation.y = q[2]
    pose_msg.pose.pose.orientation.z = q[3]
    pose_msg.pose.pose.orientation.w = q[0]
    pose_msg.pose.covariance[0] = 0.25
    pose_msg.pose.covariance[7] = 0.25
    pose_msg.pose.covariance[35] = math.radians(10) ** 2
    time.sleep(1.0)  
    pub.publish(pose_msg)
    node.get_logger().info(f"📍 初始位置已設定為 ({x}, {y}, {yaw_deg}°)")
    return {'x': q[1], 'y': q[2], 'z': q[3], 'w': q[0]}



def initialize_ros_node():
    if "ros_node" not in st.session_state or st.session_state["ros_node"] is None:
        try:
            if not rclpy.ok():
                rclpy.init()

            node = SmartNavNode()
            st.session_state["ros_node"] = node

            def spin_node():
                try:
                    rclpy.spin(node)
                except Exception as e:
                    print(f"ROS 節點錯誤：{str(e)}")

            ros_thread = threading.Thread(target=spin_node, daemon=True)
            ros_thread.start()
            st.session_state["ros_thread"] = ros_thread

            return True
        except Exception as e:
            st.error(f"ROS 節點初始化失敗：{str(e)}")
            return False
    return True

st.set_page_config(page_title="Advantech AMR 控制", layout="wide")

col1, col2 = st.columns([4, 1]) 

with col1:
    st.markdown("""
    <style>
    .title-glow {
        font-family: 'Segoe UI', sans-serif;
        font-weight: 800;
        font-size: 42px;
        text-align: left;
        background: linear-gradient(90deg, #a855f7, #ec4899, #a855f7);
        background-size: 300% 100%;
        -webkit-background-clip: text;
        -webkit-text-fill-color: transparent;
        animation: shine 6s linear infinite;
        letter-spacing: 1px;
    }

    @keyframes shine {
        0% { background-position: 0% 50%; }
        50% { background-position: 100% 50%; }
        100% { background-position: 0% 50%; }
    }
    </style>

    <h1 class="title-glow">AMR Advantech</h1>
    """, unsafe_allow_html=True)

with col2:
    with open("/home/amr/Desktop/redhat.png", "rb") as f:
        img_data_redhat = f.read()
    img_base64_redhat = base64.b64encode(img_data_redhat).decode()


    with open("/home/amr/Desktop/Intel.png", "rb") as f:
        img_data_intel = f.read()
    img_base64_intel = base64.b64encode(img_data_intel).decode()
    
    st.markdown(
        f"""
        <div style="
            display: flex; 
            justify-content: center; 
            align-items: center; 
            height: 100px;              /* 調整高度與標題匹配 */
            margin-top: 0px;           /* 移除上邊距 */
        ">
            <img src="data:image/png;base64,{img_base64_redhat}" width="120">
            <span style="font-size: 28px; margin: 0 15px;">|</span>
            <img src="data:image/png;base64,{img_base64_intel}" width="120">
        </div>
        """,
        unsafe_allow_html=True
    )

 

st.sidebar.title(t["sidebar_title"])

if st.session_state.get("is_logged_in"):
    st.sidebar.caption(f"{t['welcome_user']}{st.session_state.get('username', '')}{t['honorific']}")

page = st.sidebar.radio(" ", t["sidebar_pages"])
if page == t["sidebar_pages"][2]:  
    st.sidebar.markdown("---")  
    nav_mode = st.sidebar.selectbox("導航模式", t["nav_modes"], key="nav_mode_selector", label_visibility="collapsed")
    st.session_state["nav_mode"] = nav_mode
    st.sidebar.markdown("---")



st.sidebar.markdown("<br><br><br><br><br><br><br><br><br><br><br>", unsafe_allow_html=True)

col1, col2 = st.sidebar.columns(2)
with col1:
    login_clicked = st.button(t["login"], disabled=st.session_state.get("is_logged_in", False))
with col2:
    register_clicked = st.button(t["register"])

modal_login = Modal("🔐 使用者登入", key="modal_login")
modal_register = Modal("🆕 註冊帳號", key="modal_register")

if login_clicked:
    modal_login.open()
    st.session_state.login_modal = True
    st.session_state.register_modal = False
if register_clicked:
    modal_register.open()
    st.session_state.register_modal = True
    st.session_state.login_modal = False

if modal_login.is_open():
    with modal_login.container():
        username = st.text_input(t["login_account"], key="login_user")
        password = st.text_input(t["login_password"], type="password", key="login_pass")
        if st.button(t["login_button"], key="login"):
            if username == "amazon" and password == "amazon":
                st.session_state.is_logged_in = True
                st.session_state.username = username
                st.success(f"{t['login_success']}{username}~")
                time.sleep(2)
                modal_login.close()
                st.session_state.login_modal = False
            else:
                st.error(t["login_error"])

if modal_register.is_open():
    with modal_register.container():
        st.text_input(t["register_account"])
        st.text_input(t["register_password"], type="password")
        st.text_input(t["register_password2"], type="password")
        if st.button(t["register_button"], key="register"):
            st.success(t["register_success"])
            time.sleep(2)
            modal_register.close()
            st.session_state.register_modal = False

if page != t["sidebar_pages"][0] and not st.session_state.get("is_logged_in", False):
    st.error("⚠️ " + t["error_login_required"])
    st.session_state["force_to_home"] = True
    page = t["sidebar_pages"][0]

if st.session_state.get("force_to_home"):
    st.session_state["force_to_home"] = False

if page == t["sidebar_pages"][0]:
    st.subheader(t["hardware_title"])
    col1, col2 = st.columns([2, 1])
    with col1:
        st.markdown(f"""
            <style>
            .custom-table {{
            border-collapse: collapse;
            width: 90%;
            font-size: 15px;
            table-layout: fixed;
            }}
            .custom-table th, .custom-table td {{
            border: 1px solid #555;
            padding: 10px;
            text-align: left;
            vertical-align: top;
            word-break: break-word;
            }}
            .custom-table th {{
            background-color: #444;
            color: white;
            }}
            .custom-table td {{
            background-color: #2e2e2e;
            color: #f0f0f0;
            }}
            .custom-table tr:nth-child(even) td {{
            background-color: #3a3a3a;
            }}
            </style>

            <table class="custom-table">
            <tr>
                <th>{t["hardware_spec_item"]}</th>
                <th>{t["hardware_spec_description"]}</th>
                <th>{t["hardware_spec_local"]}</th>
            </tr>
            <tr>
                <td>{t["hardware_cpu"]}</td>
                <td>{t["hardware_cpu_desc"]}</td>
                <td>Intel Core i7-13700E</td>
            </tr>
            <tr>
                <td>{t["hardware_gpu"]}</td>
                <td>{t["hardware_gpu_desc"]}</td>
                <td>Intel UHD Graphics 770 (Raptor Lake)</td>
            </tr>
            <tr>
                <td>{t["hardware_ram"]}</td>
                <td>{t["hardware_ram_desc"]}</td>
                <td>32GB DDR4 </td>
            </tr>
            <tr>
                <td>{t["hardware_storage"]}</td>
                <td>{t["hardware_storage_desc"]}</td>
                <td>512GB NVMe SSD</td>
            </tr>
            <tr>
                <td>{t["hardware_network"]}</td>
                <td>{t["hardware_network_desc"]}</td>
                <td>4x GbE (eno1, eno2, enp4s0, enp5s0) + Wi-Fi (wlp3s0) + CAN Bus (can0)</td>
            </tr>
            <tr>
                <td>{t["hardware_io"]}</td>
                <td>{t["hardware_io_desc"]}</td>
                <td>4x USB 3.2, HDMI, DP, 4x GbE, Wi-Fi, CAN Bus</td>
            </tr>
            <tr>
                <td>{t["hardware_temp"]}</td>
                <td>{t["hardware_temp_desc"]}</td>
                <td></td>
            </tr>
            <tr>
                <td>{t["hardware_expansion"]}</td>
                <td>{t["hardware_expansion_desc"]}</td>
                <td>{t["hardware_expansion_local"]}</td>
            </tr>
            </table>
            """, unsafe_allow_html=True)
        
        with col2:
            st.image("/home/amr/Desktop/robot_code/ros2_openvino_toolkit/script/amr.png", caption="",  use_container_width=True)
    
    st.subheader(t["chatbot_title"])
    client = OpenAI(api_key="sk-proj-b6OYuNN2osgfhdupnH3UT3BlbkFJykCrG7wKwoDsqsxKs7M6")
    if "chat_history" not in st.session_state:
        st.session_state.chat_history = []

    if "input_key_id" not in st.session_state:
        st.session_state.input_key_id = 0

    input_key = f"user_input_{st.session_state.input_key_id}"
    user_input = st.text_input(t["chatbot_input"], key=input_key)

    chat_container = st.container()

    def get_bot_reply(user_message):
        try:
            response = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": t["chatbot_system_prompt"]},
                    *st.session_state.chat_history,
                    {"role": "user", "content": user_message}
                ],
                temperature=0.7,
                max_tokens=300
            )
            return response.choices[0].message.content
        except Exception as e:
            return f"{t['chatbot_error']}{e}"

    if user_input:
        st.session_state.chat_history.append({"role": "user", "content": user_input})
        reply = get_bot_reply(user_input)
        st.session_state.chat_history.append({"role": "assistant", "content": reply})

    if st.button(t["chatbot_clear"]):
        st.session_state.chat_history.clear()
        st.session_state.input_key_id += 1 

    with chat_container:
        for i, chat in enumerate(st.session_state.chat_history[-12:]):
            is_user = chat["role"] == "user"
            message(chat["content"], is_user=is_user, key=f"chat_{i}")
    # user_input = st.text_input("💬 你想說什麼？", key="user_input")

    # def get_bot_reply(user_message):
    #     user_message = user_message.lower()
    #     if "hello" in user_message:
    #         return "你好！有什麼我可以幫忙的？"
    #     elif "battery" in user_message:
    #         return "目前電量為 30%，建議儘快返回充電站 🔋"
    #     elif "navigation" in user_message:
    #         return "正在規劃導航路徑，請稍後..."
    #     else:
    #         return "抱歉，我還聽不懂這句話 😅"

    # if user_input:
    #     st.session_state.chat_history.append({"role": "user", "content": user_input})
    #     bot_reply = get_bot_reply(user_input)
    #     st.session_state.chat_history.append({"role": "bot", "content": bot_reply})

    # latest_chats = st.session_state.chat_history[-6:]

    # for i, chat in enumerate(latest_chats):
    #     is_user = chat["role"] == "user"
    #     message(chat["content"], is_user=is_user, key=f"chat_{i}")


elif page == t["sidebar_pages"][1]:
    st.subheader(t["radar_title_1"])
    if "radar_started" not in st.session_state:
        st.session_state["radar_started"] = False

    st.info(t["radar_info_1"])
    st.code("""
    cd ~/Documents/ros2_amr/AMR_script && ./run_2D_SLAM.sh
    """, language="bash")
    command = st.text_input(t["radar_input_command"], key="open")

    if st.button(t["radar_execute"], key="run_open_command"):
        if command.strip():
            result = subprocess.Popen(command, shell=True)
            st.session_state["radar_started"] = True
            st.code(result.stdout or t["radar_success"])
            if result.stderr:
                st.error(result.stderr)
        else:
            st.warning(t["radar_input_warning"])

    st.subheader(t["radar_title_2"])
    rviz_script_path = "/home/amr/Desktop/robot_code/ros2_openvino_toolkit/script/rviz.py"
    image_path = "/home/amr/Desktop/robot_code/rvizslam/rviz_snap.png"

    def wait_for_valid_recent_image(path, max_age=2, timeout=2, interval=0.2):
        """等待一張最近 `max_age` 秒內更新過且可以開啟的圖片"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if os.path.exists(path) and os.path.getsize(path) > 1000:
                modified_time = os.path.getmtime(path)
                if time.time() - modified_time <= max_age:
                    try:
                        img = Image.open(path)
                        img.verify()
                        return Image.open(path)
                    except UnidentifiedImageError:
                        time.sleep(interval)
                else:
                    time.sleep(interval)
            else:
                time.sleep(interval)
        return None

    col1, col2 = st.columns(2)

    with col1:
        if st.session_state["radar_started"]:
            if st.button(t["radar_show_button"], key="show_rviz"):
                subprocess.Popen(["python3", rviz_script_path])
                st.success(t["radar_show_success"])
        else:
            st.button(t["radar_show_button"], key="show_rviz_disabled", disabled=True)
    
    with col2:
        if st.session_state["radar_started"]:
            if st.button(t["radar_stop_button"], key="stop_rviz"):
                os.system("pkill -f rviz.py")
                st.warning(t["radar_stop_success"])
        else:
            st.button(t["radar_stop_button"], key="stop_rviz", disabled=True)
    image_container = st.empty()

    img = wait_for_valid_recent_image(image_path, max_age=2, timeout=2)
    if img:
        image_container.image(img, caption=t["radar_image_caption"], use_container_width=True)
    else:
        st.warning(t["radar_no_image"])

    st.subheader(t["radar_title_3"])
    st.info(t["radar_info_3"])
    st.code("""
    cd ~/Documents/ros2_amr/AMR_script && ./save_map.sh && ./stop_2D_SLAM.sh
    """, language="bash")
    command_close = st.text_input(t["radar_input_command"], key="close")
    if st.button(t["radar_execute"], key="run_close_command"):
        if command_close.strip():
            result = subprocess.run(command_close, shell=True, capture_output=True, text=True)
            st.code(result.stdout or t["radar_no_output"])
            if result.stderr:
                st.error(result.stderr)
        else:
            st.warning(t["radar_input_warning"])



elif page == t["sidebar_pages"][2]:
    nav_mode = st.session_state.get("nav_mode_selector", t["nav_modes"][0])
    if nav_mode == t["nav_modes"][0]:
        st.subheader(t["env_init_title"])
        st.info(t["env_init_info"])
        st.code("""
        cd ~/Documents/ros2_amr/AMR_script && ./run_navigation.sh
        """, language="bash")
        command_input = st.text_input(t["input_command"], key="build_nav_environment")
        if st.button(t["execute_button"], key="run_nav_command"):
            if command_input.strip() == "":
                st.warning(t["input_warning"])
            else:
                try:
                    process = subprocess.Popen(
                        command_input,
                        shell=True,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        preexec_fn=os.setsid
                    )
                    st.session_state["nav_proc"] = process
                    st.success(t["execute_success"].format(process.pid))
                except Exception as e:
                    st.error(t["execute_error"].format(str(e)))

        st.subheader(t["nav_task_title"])
        if "goal_points" not in st.session_state:
            st.session_state.goal_points = [{"x": 0.0, "y": 0.0, "yaw": 0.0}]

        if st.button(t["start_nav_node"], key="start_navigation_node"):
            if initialize_ros_node():
                st.success(t["nav_node_success"])
            else:
                st.info(t["nav_node_info"])


        def publish_initial_pose(node, x=0.0, y=0.0, yaw_deg=0.0):
            pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            q = euler2quat(0, 0, math.radians(yaw_deg))
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.orientation.x = q[1]
            pose_msg.pose.pose.orientation.y = q[2]
            pose_msg.pose.pose.orientation.z = q[3]
            pose_msg.pose.pose.orientation.w = q[0]
            pose_msg.pose.covariance[0] = 0.25
            pose_msg.pose.covariance[7] = 0.25
            pose_msg.pose.covariance[35] = math.radians(10) ** 2
            time.sleep(1.0)  
            pub.publish(pose_msg)
            node.get_logger().info(f"📍 初始位置已設定為 ({x}, {y}, {yaw_deg}°)")
            return {'x': q[1], 'y': q[2], 'z': q[3], 'w': q[0]}

        #st.markdown("---")

        @st.dialog(t["nav_dialog_title"])
        def show_navigation_dialog():
            with st.form("navigation_form_in_dialog"):
                st.markdown(f"#### {t['start_coord_title']}")
                col1, col2, col3 = st.columns(3)
                with col1:
                    start_x = st.number_input(t["start_x"], key="start_x", format="%.2f")
                with col2:
                    start_y = st.number_input(t["start_y"], key="start_y", format="%.2f")
                with col3:
                    start_yaw = st.number_input(t["start_yaw"], key="start_yaw", format="%.2f")

                set_start_clicked = st.form_submit_button(t["set_start_button"])
                if set_start_clicked:
                    st.session_state.start_pose = {
                        "x": start_x,
                        "y": start_y,
                        "yaw": start_yaw
                    }
                    if "ros_node" in st.session_state:
                        publish_initial_pose(
                            node=st.session_state.ros_node,
                            x=start_x,
                            y=start_y,
                            yaw_deg=start_yaw
                        )
                        st.success(t["start_set_success"].format(start_x, start_y, start_yaw))
                    else:
                        st.warning(t["ros_node_warning"])
                        
                st.markdown("---")


                st.markdown(f"#### {t['goal_coord_title']}")
                for i, point in enumerate(st.session_state.goal_points):
                    st.markdown(f"#### {t['goal_group'].format(i+1)}")
                    col4, col5, col6 = st.columns(3)
                    with col4:
                        st.session_state.goal_points[i]["x"] = st.number_input(
                            t["goal_x"].format(i+1), key=f"goal_x_{i}", value=point["x"], format="%.2f")
                    with col5:
                        st.session_state.goal_points[i]["y"] = st.number_input(
                            t["goal_y"].format(i+1), key=f"goal_y_{i}", value=point["y"], format="%.2f")
                    with col6:
                        st.session_state.goal_points[i]["yaw"] = st.number_input(
                            t["goal_yaw"].format(i+1), key=f"goal_yaw_{i}", value=point["yaw"], format="%.2f")

                col_add, col_send = st.columns([1, 1])
                with col_add:
                    add_clicked = st.form_submit_button(t["add_goal_button"])
                with col_send:
                    send_clicked = st.form_submit_button(t["send_nav_button"])

                if add_clicked:
                    st.session_state.goal_points.append({"x": 0.0, "y": 0.0, "yaw": 0.0})

                if send_clicked:
                    if "ros_node" not in st.session_state:
                        st.error(t["ros_node_error"])
                        return

                    goals = [
                        (point["x"], point["y"], point["yaw"])
                        for point in st.session_state.goal_points
                    ]
                    ros_node: SimpleNavNode = st.session_state.ros_node
                    ros_node.set_goal_queue(goals)
                    ros_node.start_navigation()
                    st.success(t["nav_task_success"])
                    st.session_state.show_dialog = False
                    st.rerun()

        st.markdown("""
            <style>
            div[data-testid="stButton"] > button {
                border: none;
                background: none;
                color: #1f77b4;
                padding: 0;
                font-size: 16px;
                cursor: pointer;
            }
            </style>
            """, unsafe_allow_html=True)
        if st.button(t["open_nav_dialog"]):
            show_navigation_dialog()

        st.subheader(t["nav_status_title"])
        
        status_container = st.container()
        
        @st.fragment(run_every=1)  
        def update_navigation_status():
            def load_ui_status():
                try:
                    with open("/home/amr/Desktop/robot_code/ui_status/ui_status.json", "r") as f:
                        return json.load(f)
                except:
                    return {
                        "total_goals": 0,
                        "current_goal_index": 0,
                        "navigation_status": t["status_paused"]
                    }
            def load_yolo_status():
                try:
                    with open("/home/amr/Desktop/robot_code/ui_status/yolo_status.json", "r") as f:
                        return json.load(f)
                except:
                    return {"YOLO偵測結果": []}

            data = load_ui_status()
            total_goals = data["total_goals"]
            current_goal_index = data["current_goal_index"]
            navigation_status = data["navigation_status"]
            status_color = {
                "進行中": "#28a745",   
                "避障中": "#b8860b",   
                "暫停中": "#595959"   
            }
            color = status_color.get(navigation_status, "#ffffff")

            col1, col2, col3 = st.columns(3)

            with col1:
                actual_total = total_goals - 1 if total_goals > 0 else 0
                st.success(t["total_goals"].format(actual_total))

            with col2:
                actual_total = total_goals - 1 if total_goals > 0 else 0
        
                if current_goal_index == 0:
                    st.success(t["current_goal_ready"])
                elif current_goal_index <= actual_total:
                    st.success(t["current_goal_progress"].format(current_goal_index, actual_total))
                else:
                    st.success(t["current_goal_return"])

            with col3:
                st.markdown(
                    f"<div style='background-color:{color}; padding:10px; border-radius:8px; color:white; height:55px; display:flex;align-items:center;left:10px'>"
                    f"<strong>{t['nav_status_label']} {navigation_status}</div>",
                    unsafe_allow_html=True
                )

            yolo_data = load_yolo_status().get("YOLO偵測結果", [])

            if yolo_data:
                st.subheader(t["yolo_title"])
                st.table(yolo_data)
            else:
                st.info(t["yolo_no_detection"])
        
        update_navigation_status()

        st.subheader(t["seg_title"])
        image_placeholder = st.empty()
        if "ros_node" in st.session_state and st.session_state["ros_node"] is not None:
            nav_node = st.session_state["ros_node"]

            @st.fragment(run_every=1)
            def update_segmentation_image():
                with nav_node.segmentation_lock:
                    seg_image = nav_node.latest_segmented_image

                if seg_image is not None:
                    seg_rgb = cv2.cvtColor(seg_image, cv2.COLOR_BGR2RGB)
                    seg_pil = Image.fromarray(seg_rgb)
                    image_placeholder.image(seg_pil, caption=t["semantic_caption"], use_container_width=True)
                else:
                    image_placeholder.info(t["waiting_seg"])

            update_segmentation_image()

        else:
            st.warning(t["ros_not_ready"])


        st.subheader(t["end_task_title"])
        if st.button(t["close_nav_button"], key="run_close_command"):
            try:
                try:
                    nav_node.stop_loop_speech() 
                    nav_node.warning_speech_active = False  
                    if nav_node.warning_speech_thread is not None:
                        nav_node.warning_speech_thread.join(timeout=1.0)
                        nav_node.warning_speech_thread = None

                    nav_node.stop_camera_subscription()
                    nav_node.get_logger().info("🔇 已停止所有語音播報")
                except Exception as e:
                    print(f"⚠️ 停止語音失敗: {e}")
                stop_command = "cd ~/Documents/ros2_amr/AMR_script && ./stop_navigation.sh"
                result1 = subprocess.run(stop_command, shell=True, capture_output=True, text=True)

                kill_command = "pkill -f smart_nav_node.py"
                result2 = subprocess.run(kill_command, shell=True, capture_output=True, text=True)

                st.success(t["close_nav_success"])
                st.code(result1.stdout + "\n" + result2.stdout or t["no_output"])
                if result1.stderr or result2.stderr:
                    st.error(t["error_output"].format(result1.stderr, result2.stderr))
            except Exception as e:
                st.error(t["execute_failed"].format(str(e)))

    elif nav_mode == t["nav_modes"][1]:
        col_a, col_b = st.columns([1.2, 1.8])

        with col_a:
            st.text(t["control_panel"])
            st.markdown("---")

            # ---- 步驟 1 ----
            st.text(t["nav_step1_title"])
            if st.button(t["nav_step1_button"], use_container_width=True):
                with st.spinner(t["nav_step1_loading"]):
                    try:
                        rviz_proc = subprocess.Popen(
                            "cd ~/Documents/ros2_amr/AMR_script && ./run_navigation.sh",
                            shell=True, executable="/bin/bash",
                        )
                        st.session_state["rviz_pid"] = rviz_proc.pid
                        time.sleep(8)
                        st.success(t["nav_step1_success"])
                    except Exception as e:
                        st.error(t["nav_step1_error"].format(error=e))

            # ---- 步驟 2 ----
            st.text(t["nav_step2_title"])
            if st.button(t["nav_step2_button"], use_container_width=True):
                with st.spinner(t["nav_step2_loading"]):
                    try:
                        cmd = (
                            "cd /home/amr/Desktop/robot_code/ros2_openvino_toolkit/script && "
                            "nohup python3 testgranitenav.py > /home/amr/Desktop/robot_code/semantic_nav.log 2>&1 &"
                        )
                        subprocess.Popen(cmd, shell=True, executable="/bin/bash")

                        log_path = "/home/amr/Desktop/robot_code/semantic_nav.log"
                        start_time = time.time()
                        success_flag = False
                        progress_placeholder = st.empty()

                        while time.time() - start_time < 20:  
                            if os.path.exists(log_path):
                                with open(log_path, "r") as f:
                                    lines = f.readlines()
                                    for line in lines[-10:]:  
                                        if "Loading checkpoint shards: 100%" in line or "ZeroMQ 接收器已啟動" in line:
                                            st.success(t["nav_step2_success"])
                                            #st.info("📄 詳細日誌可在終端查看： tail -f /home/amr/Desktop/robot_code/semantic_nav.log")
                                            success_flag = True
                                            break
                                    if success_flag:
                                        break
                            time.sleep(1)

                        if not success_flag:
                            st.warning(t["nav_step2_warning"])

                    except Exception as e:
                        st.error(t["nav_step2_error"])


            # ---- 步驟 3 ----
            st.text(t["nav_step3_title"])
            if st.button(t["nav_step3_button"], use_container_width=True):
                with st.spinner(t["nav_step3_loading"]):
                    try:
                        subprocess.Popen(
                            "python3 /home/amr/Desktop/robot_code/ros2_openvino_toolkit/script/set_initial_pose.py",
                            shell=True, executable="/bin/bash",
                        )
                        st.success(t["nav_step3_success"])
                    except Exception as e:
                        st.error(t["nav_step3_error"])

            # ---- 步驟 4 ----
            st.text(t["nav_step4_title"])
            model = st.selectbox(
                " ",
                [t["nav_step4_select"], "BLIP（CV Model）", "Granite（NLP Model）", "BLIP + Granite（Hybrid）"],
                index=0,
            )
            if model != t["nav_step4_select"]:
                st.success(t["nav_step4_success"].format(model=model))

            # ---- 步驟 5 ----
            st.text(t["nav_step5_title"])
            task = st.text_area("輸入任務內容", placeholder=t["nav_step5_placeholder"], label_visibility="collapsed")
            if st.button(t["nav_step5_button"], use_container_width=True):
                if task.strip():
                    try:
                        context = zmq.Context()
                        socket = context.socket(zmq.PUSH)
                        socket.connect("tcp://127.0.0.1:5555")
                        socket.send_string(f"模型選擇：{model}")
                        time.sleep(0.2)
                        socket.send_string(task)
                        socket.close()
                        st.success(t["nav_step5_success"].format(task=task))
                    except Exception as e:
                        st.error(t["nav_step5_error"].format(error=e))
                else:
                    st.warning(t["nav_step5_warning"])

            # ---- 步驟 6 ----
            st.text(t["nav_step6_title"])
            if st.button(t["nav_step6_button"], use_container_width=True):
                with st.spinner(t["nav_step6_loading"]):
                    try:
                        success_msgs = []
                        # 🧩 1️⃣ 關閉 Rviz2 與啟動腳本
                        stop_nav_cmd = "cd ~/Documents/ros2_amr/AMR_script && ./stop_navigation.sh"
                        result = subprocess.run(stop_nav_cmd, shell=True, capture_output=True, text=True)

                        # 🧩 2️⃣ 關閉語義導航後端（Granite + YOLO）
                        subprocess.run("pkill -f testgranitenav.py", shell=True)


                        if result.returncode == 0:
                            st.success(t["nav_step6_success"])
                        else:
                            st.warning(t["nav_step6_warning"].format(warn=result.stderr))

                        log_path = "/home/amr/Desktop/robot_code/semantic_nav.log"
                        if os.path.exists(log_path):
                            os.remove(log_path)
                            success_msgs.append("🧹 已清理暫存日誌檔案 /home/amr/Desktop/robot_code/semantic_nav.log")
                    except Exception as e:
                        st.error(t["nav_step6_error"])

        with col_b:
            st.text(t["nav_task_order"])
            vue_html = """
                <!DOCTYPE html>
                <html>
                <head>
                    <meta charset="UTF-8">
                    <title>Vue Timeline</title>
                    <!-- Element Plus CSS -->
                    <link rel="stylesheet" href="https://unpkg.com/element-plus/dist/index.css">
                    <!-- Vue 3 + Element Plus -->
                    <script src="https://unpkg.com/vue@3"></script>
                    <script src="https://unpkg.com/element-plus"></script>
                    <style>
                        body {
                            background-color: transparent;
                            color: #e5e7eb;
                            font-family: "Inter", "Noto Sans TC", sans-serif;
                            margin-top: 35px;
                        }
                        .el-timeline {
                            padding-left: 10px;
                            color: #e5e7eb;
                        }
                        .el-timeline-item__node {
                            background-color: #8b5cf6/* 預設紫色 */
                            transition: background-color 0.4s;
                        }
                        .el-timeline-item__node.active {
                            background-color: #22c55e !important; /* 當前步驟變綠色 */
                        }
                        .el-timeline-item__content {
                            color: #e5e7eb;
                        }
                        .process {
                            background-color: #2b2b3c !important;
                            border: none !important;
                            border-radius: 10px;
                            color: #e5e7eb;
                            transition: all 0.3s ease;
                            padding: 10px;
                        }
                        .process:hover {
                            transform: translateY(-3px);
                            box-shadow: 0 4px 10px rgba(0,0,0,0.4);
                        }
                        .detect-box {
                            margin-top: 25px;
                            text-align: left;
                            font-size: 16px;
                            color: #a5b4fc;
                            font-weight: 500;
                            transition: all 0.3s ease;
                        }
                    </style>
                </head>
                <body>
                    <div id="app">
                        <el-timeline v-if="steps.length > 0" class="timeline-container">
                            <el-timeline-item
                                v-for="(item, idx) in steps"
                                :key="idx"
                                :timestamp="item.type"
                                placement="top"
                                :color="idx === currentStep ? '#22c55e' : '#8b5cf6'"
                            >
                                {{ item.detail }}
                            </el-timeline-item>
                        </el-timeline>

                        <div v-else class="process">
                            <span>🚩 No task data yet or navigation has ended</span>
                        </div>

                        <div v-if="detect_result" class="detect-box">
                            {{ detect_result }}
                        </div>
                        <div v-else class="detect-box" style="opacity:0.5;">
                            
                        </div>
                    </div>

                    <script>
                    const { createApp, ref, onMounted } = Vue

                    createApp({
                        setup() {
                            const steps = ref([])
                            const detect_result = ref(null)
                            const currentStep = ref(null)

                            async function fetchTimeline() {
                                try {
                                    const res = await fetch("http://127.0.0.1:5000/timeline?nocache=" + Date.now())
                                    const data = await res.json()
                                    if (data.status === "ok") {
                                        steps.value = data.steps || []
                                        detect_result.value = data.detect_result
                                        currentStep.value = data.current_step ?? null
                                    } else if (data.status === "finished" || data.status === "no_log") {
                                        steps.value = []
                                        detect_result.value = null
                                    }
                                } catch (e) {
                                    console.log("⏳ 等待 Flask 傳回資料中...")
                                }
                            }

                            onMounted(() => {
                                fetchTimeline()
                                setInterval(fetchTimeline, 2000)
                            })

                            return { steps, detect_result, currentStep }
                        }
                    }).use(ElementPlus).mount('#app')
                    </script>
                </body>
                </html>
                """
            components.html(vue_html, height=600, scrolling=True)
        
        st.markdown('')
        st.markdown('')

        st.text(t["model_response"])
        json_path = "/home/amr/Desktop/robot_code/granite_picture/summary.json"
        latest_data = None
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
                if isinstance(data, dict):
                    data = [data]  
                if isinstance(data, list) and data:
                    latest_data = data
        except Exception as e:
            pass
            #st.warning(f"無法載入 JSON: {e}")

        latest_json = json.dumps(latest_data, ensure_ascii=False) if latest_data else "[]"

        vue_html = f"""
        <!DOCTYPE html>
        <html>
        <head>
        <meta charset="UTF-8">
        <title>最新導航結果</title>
        <link rel="stylesheet" href="https://unpkg.com/element-plus/dist/index.css">
        <script src="https://unpkg.com/vue@3"></script>
        <script src="https://unpkg.com/element-plus"></script>

        <style>
            body {{
            background-color: transparent;
            color: #e5e7eb;
            font-family: "Inter", "Noto Sans TC", sans-serif;
            margin: 0;
            padding: 10px;
            }}
            ::-webkit-scrollbar {{
            width: 0px;
            background: transparent;
            }}
            .flex {{
            display: flex;
            flex-wrap: wrap;
            gap: 16px;
            justify-content: flex-start;
            }}
            .el-card {{
            background-color: #2b2b3c !important;
            border: none !important;
            border-radius: 10px;
            color: #e5e7eb;
            width: 520px;
            transition: all 0.3s ease;
            }}
            .el-card:hover {{
            transform: translateY(-3px);
            box-shadow: 0 4px 10px rgba(0,0,0,0.4);
            }}
            .preview-image {{
                width: 100%;
                height: 230px;              /* 固定高度，統一比例 */
                object-fit: cover;          /* 不變形 */
                border-radius: 10px;
                background-color: #0f0f0f;  /* 背景填充色 */
                box-shadow: 0 3px 10px rgba(0, 0, 0, 0.3);
                margin-bottom: 12px;
            }}
            .time{{
                font-weight: 600;
                font-size: 15px;
                color: #c7d2fe;
                margin-bottom: 10px;
            }}
            .caption {{
            font-weight: 600;
            font-size: 15px;
            color: #c7d2fe;
            margin-top: 10px;
            }}
            .description {{
            font-weight: 600;
            font-size: 15px;
            color: #c7d2fe;
            line-height: 1.6;
            margin-top: 8px;
            text-align: justify;
            }}
        </style>
        </head>

        <body>
        <div id="app">
            <div class="flex">
            <template v-if="results.length > 0">
                <el-card
                    v-for="(item, idx) in results"
                    :key="idx"
                    shadow="always"
                >
                    <div class="time">⏰ 生成時間：{{{{ item.generated_time }}}}</div>
                    <img
                        v-if="item.filename"
                        :src="'http://127.0.0.1:5000/images/' + item.filename"
                        alt="Captured image"
                        class="preview-image"
                    />
                    <div class="caption">🪶 BLIP解析：{{{{ item.blip_caption }}}}</div>
                    <div class="description">📘 Granite生成：{{{{ item.description }}}}</div>
                </el-card>
            </template>
            <el-card shadow="never" v-else>
                <p style="color:gray;text-align: center;">No response received yet</p>
            </el-card>
            </div>
        </div>

        <script>
            const {{ createApp, ref, onMounted }} = Vue
            createApp({{
            setup() {{
                const results = ref([])
                onMounted(() => {{
                    setInterval(async () => {{
                        try {{
                        const res = await fetch("http://127.0.0.1:5000/data?nocache=" + Date.now())
                        if (!res.ok) return
                        const data = await res.json()
                        const list = data.model_results || []
                        if (Array.isArray(list) && list.length > 0) {{
                            const now = new Date()
                            const valid = list.filter(item => {{
                            if (!item.generated_time) return false
                            const genTime = new Date(item.generated_time)
                            const diffHours = (now - genTime) / (1000 * 60 * 60)
                            return diffHours <= 2 
                            }})

                            results.value = valid.reverse()
                        }}
                        }} catch (e) {{
                        console.log("⏳ 等待 Flask 傳回資料中...")
                        }}
                    }}, 2000)
                }})
                return {{ results }}
            }}
            }}).use(ElementPlus).mount('#app')
        </script>
        </body>
        </html>
        """

        components.html(vue_html, height=500, scrolling=True)


elif page == t["sidebar_pages"][3]:
    st.subheader(f"🔧 1. {t['camera_env_init']}")
    st.info(t["camera_env_info"])
    st.code("""
    cd ~/Downloads/Adv_AMR_installer_v1.0.0/AMR_script/sh && ./open_camera.sh && ./open_tracer_mini.sh
    """, language="bash")
    if "camera_proc" not in st.session_state:
        st.session_state.camera_proc = None
    command_close = st.text_input(f"{t['camera_input_command']}:", key="close")
    if st.button(f"🚀 {t['camera_execute']}", key="run_camera"):
        try:
            process = subprocess.Popen(
            command_close,
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
            )
            st.success(f"✅ {t['camera_command_success']}{process.pid}")
        except Exception as e:
            st.error(f"❌ {t['camera_error']}{str(e)}")
    
    st.code("""
    python3 /home/amr/Desktop/robot_code/ros2_openvino_toolkit/script/robotCamera.py
    """, language="bash")

    if "python_proc" not in st.session_state:
        st.session_state.python_proc = None

    command_py = st.text_input(f"{t['camera_input_python']}:", key="py_command")

    if st.button(f"🚀 {t['camera_execute']}", key="run_python"):
        try:
            process_py = subprocess.Popen(
                command_py,
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            st.session_state.python_proc = process_py
            st.success(f"✅ {t['camera_python_success']}{process_py.pid}")
        except Exception as e:
            st.error(f"❌ {t['camera_error']}{str(e)}")
    
    st.subheader(f"📷 2. {t['camera_view_title']}")
    camera_image_path = "/home/amr/Desktop/robot_code/camera/frame.jpg"
    st.caption(t["camera_view_caption"])

    if "camera_on" not in st.session_state:
        st.session_state.camera_on = False

    col1, col2 = st.columns([1, 1])
    with col1:
        if st.button(f"▶️ {t['camera_start']}"):
            st.session_state.camera_on = True
    with col2:
        if st.button(f"⏹️ {t['camera_stop']}"):
            st.session_state.camera_on = False

            try:
                os.killpg(os.getpgid(st.session_state.camera_proc.pid), signal.SIGTERM)
                st.success(f"🛑 {t['camera_script_closed']}")
            except Exception as e:
                st.warning(f"⚠️ {t['camera_script_close_error']}{e}")
            st.session_state.camera_proc = None

            try:
                os.killpg(os.getpgid(st.session_state.python_proc.pid), signal.SIGTERM)
                st.success(f"🛑 {t['camera_python_closed']}")
            except Exception as e:
                st.warning(f"⚠️ {t['camera_python_close_error']}{e}")
            st.session_state.python_proc = None

    st.markdown(
        f"**{t['camera_status']}** {'🟢 ' + t['camera_status_on'] if st.session_state.camera_on else '🔴 ' + t['camera_status_off']}"
    )


    frame_container = st.empty()
    if st.session_state.camera_on:
        st_autorefresh(interval=2000, key="camera-refresh")

        if os.path.exists(camera_image_path):
            img = Image.open(camera_image_path)
            frame_container.image(img, caption=t["camera_image_caption"], use_container_width=True)
        else:
            frame_container.warning(f"❗ {t['camera_no_image']}")
    else:
        frame_container.empty()

    st.subheader(f"🕹️ {t['camera_control_title']}")
    st.caption(t["camera_control_caption"])

    if "ros_initialized" not in st.session_state:
        if not rclpy.ok(): 
            rclpy.init()
        st.session_state.ros_initialized = True

    if "twist_pub" not in st.session_state:
        class WebTeleop(Node):
            def __init__(self):
                super().__init__('web_teleop')
                self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

            def send_cmd(self, linear_x, angular_z):
                msg = Twist()
                msg.linear.x = linear_x
                msg.angular.z = angular_z
                self.publisher.publish(msg)
                print(f"✅ {t['camera_send_command']} linear={linear_x:.2f}, angular={angular_z:.2f}")

        st.session_state.node = WebTeleop()
        st.session_state.twist_pub = st.session_state.node.send_cmd

    if "ros_spin_started" not in st.session_state:
        def ros_spin():
            rclpy.spin(st.session_state.node)

        spin_thread = threading.Thread(target=ros_spin, daemon=True)
        spin_thread.start()
        st.session_state.ros_spin_started = True

    st.caption(f"⚙️ {t['camera_speed_setting']}")
    speed = st.slider(t["camera_linear_speed"], 0.0, 1.0, 0.2, 0.05)
    turn = st.slider(t["camera_angular_speed"], 0.0, 1.0, 0.5, 0.05)

    st.caption(f"🎮 {t['camera_keyboard_control']}")
    col_w, _, _ = st.columns([1, 1, 1])
    with col_w:
        if st.button(f"⬆️ {t['camera_forward']}"):
            st.session_state.twist_pub(speed, 0.0)

    col_a, col_s, col_d = st.columns(3)
    with col_a:
        if st.button(f"⬅️ {t['camera_left']}"):
            st.session_state.twist_pub(0.0, turn)
    with col_s:
        if st.button(f"⬇️ {t['camera_backward']}"):
            st.session_state.twist_pub(-speed, 0.0)
    with col_d:
        if st.button(f"➡️ {t['camera_right']}"):
            st.session_state.twist_pub(0.0, -turn)

    st.markdown(f"### ⛔ {t['camera_emergency_stop']}")
    if st.button(f"⏹ {t['camera_stop_robot']}"):
        st.session_state.twist_pub(0.0, 0.0)
        st.info(f"✅ {t['camera_stop_sent']}")



elif page == t["sidebar_pages"][4]:
    st.subheader(f"📟 {t['susi_status_title']}")
    st.info(t["susi_status_info"])
    st.code("""
    python3 /home/amr/Desktop/robot_code/ros2_openvino_toolkit/script/susi.py
    """, language="bash")
    
    if 'susi_status_message' not in st.session_state:
        st.session_state.susi_status_message = None
    if 'susi_status_type' not in st.session_state:
        st.session_state.susi_status_type = None
    
    command = st.text_input(f"{t['susi_input_command']}:", key="susienbir_cmd")
    col1, col2 = st.columns([1, 1])
    
    with col1:
        if st.button(f"🚀 {t['susi_start']}", key="run_environment_command"):
            if command.strip():
                try:
                    proc = subprocess.Popen(["bash", "-c", command])
                    st.session_state.susi_process = proc
                    st.session_state.susi_status_message = f"✅ {t['susi_starting']}，PID={proc.pid}"
                    st.session_state.susi_status_type = "success"
                except Exception as e:
                    st.session_state.susi_status_message = f"❌ {t['susi_exec_error']}{e}"
                    st.session_state.susi_status_type = "error"
            else:
                st.session_state.susi_status_message = f"⚠️ {t['susi_input_warning']}"
                st.session_state.susi_status_type = "warning"

    with col2:
        if st.button(f"❌ {t['susi_stop']}", key="stop_susi_button"):
            proc = st.session_state.get("susi_process", None)
            if proc is not None and proc.poll() is None:
                try:
                    os.kill(proc.pid, signal.SIGTERM)
                    st.session_state.susi_process = None
                    st.session_state.susi_status_message = f"🛑 {t['susi_stopped']}（PID={proc.pid}）"
                    st.session_state.susi_status_type = "success"
                except Exception as e:
                    st.session_state.susi_status_message = f"❌ {t['susi_stop_error']}{e}"
                    st.session_state.susi_status_type = "error"
            else:
                st.session_state.susi_status_message = f"⚠️ {t['susi_not_running']}"
                st.session_state.susi_status_type = "warning"
    
    if st.session_state.susi_status_message:
        if st.session_state.susi_status_type == "success":
            st.success(st.session_state.susi_status_message)
        elif st.session_state.susi_status_type == "error":
            st.error(st.session_state.susi_status_message)
        elif st.session_state.susi_status_type == "warning":
            st.warning(st.session_state.susi_status_message)


    st.divider()

    def load_susi_json():
        try:
            with open("/home/amr/Desktop/robot_code/susi/susi_data.json", "r") as f:
                return json.load(f)
        except Exception as e:
            return {"error": str(e)}

    REFRESH_INTERVAL = 150 

    if 'last_update' not in st.session_state:
        st.session_state.last_update = 0 
    if 'update_counter' not in st.session_state:
        st.session_state.update_counter = 0
    if 'data' not in st.session_state:
        st.session_state.data = load_susi_json()

    st_autorefresh(interval=1000, key="check_refresh") 

    st.title(f"🧠 {t['susi_monitor_title']}")
    col1, col2 = st.columns(2)
    with col1:
        manual_refresh = st.button(f"🔄 {t['susi_manual_refresh']}", type="primary")
    with col2:
        auto_refresh_enabled = st.toggle(f"🔄 {t['susi_auto_refresh']}", value=False)

    st.markdown("---")

    NOW_TIMESTAMP = time.time()
    NOW_DATETIME = datetime.now()

    seconds_since_last_update = NOW_TIMESTAMP - st.session_state.last_update

    should_update = manual_refresh

    if should_update:
        new_data = load_susi_json()
        st.session_state.data = new_data
        
        st.session_state.last_update = NOW_TIMESTAMP
        st.session_state.update_counter += 1

        st.success("✅ 數據已手動更新！", icon="🔄")

    data = st.session_state.data

    data_container = st.container()

    with data_container:
        if "error" in data:
            st.error(f"❌ {t['susi_data_error']}{data['error']}")
        elif "system_time" not in data:
            st.error(f"❌ {t['susi_missing_time']}")
        else:
            try:
                saved_dt = datetime.strptime(data["system_time"], "%Y-%m-%d %H:%M:%S")
                time_diff = NOW_DATETIME - saved_dt
                outdated = time_diff > timedelta(minutes=2, seconds=30)
                
                col1, col2, col3 = st.columns([2, 1, 1])
                with col1:
                    st.info(f"🕒 **上次刷新** {int(seconds_since_last_update)} 秒前 | **數據時間** {saved_dt.strftime('%Y-%m-%d %H:%M:%S')}")
                with col2:
                    if outdated:
                        st.warning(f"⚠️ **{t['susi_outdated']}**")
                    else:
                        st.success(f"✅ **{t['susi_latest']}**")
                with col3:
                    st.info(f"📊 **{t['susi_monitor_items']}** {len([k for k in data.keys() if k != 'system_time']) -1 } 項")
                
                st.markdown("---")

                if outdated:
                    st.warning(t['susi_data_outdated_warning'])
                else:
                    hw_data = {}
                    for key, value in data.items():
                        if key != "system_time" and isinstance(value, dict) and "value" in value:
                            hw_data[key] = value

                    if hw_data:
                        voltage_data = {k: v for k, v in hw_data.items() if "Voltage" in k}
                        if voltage_data:
                            st.markdown(f"### {t['voltage_monitor_title']}")
                            cols = st.columns(4)
                            voltage_items = list(voltage_data.items())
                            for i, (key, value) in enumerate(voltage_items):
                                with cols[i]:
                                    name = key.split("/")[-1]
                                    voltage_val = float(value['value'])

                                    if "3.3V" in key:
                                        name += t["desc_3v"]
                                    elif "5V" in key and "Standby" not in key:
                                        name += t["desc_5v"]
                                    elif "12V" in key:
                                        name += t["desc_12v"]
                                    elif "CMOS" in key:
                                        name += t["desc_cmos"]

                                    if "3.3V" in key and (voltage_val < 3.0 or voltage_val > 3.6):
                                        st.error(f"🔋 **{name}**\n\n# {value['value']} V")
                                    elif "5V" in key and (voltage_val < 4.5 or voltage_val > 5.5):
                                        st.error(f"🔋 **{name}**\n\n# {value['value']} V")
                                    elif "12V" in key and (voltage_val < 11.0 or voltage_val > 13.0):
                                        st.error(f"🔋 **{name}**\n\n# {value['value']} V")
                                    elif "CMOS" in key and voltage_val < 2.8:
                                        st.warning(f"🔋 **{name}**\n\n# {value['value']} V")
                                    else:
                                        st.success(f"🔋 **{name}**\n\n# {value['value']} V")

                            st.markdown("<br>", unsafe_allow_html=True)

                        temp_data = {k: v for k, v in hw_data.items() if "Temperature" in k}
                        fan_data = {k: v for k, v in hw_data.items() if "Fan Speed" in k}

                        col1, col2 = st.columns(2)

                        with col1:
                            if temp_data:
                                st.markdown(f"### {t['temperature_monitor_title']}")
                                for key, value in temp_data.items():
                                    name = key.split("/")[-1]
                                    temp_val = float(value['value'])

                                    if temp_val > 80:
                                        st.error(f"🌡️ **{name} {t['temperature_label']}**\n\n## {value['value']} °C")
                                    elif temp_val > 70:
                                        st.warning(f"🌡️ **{name} {t['temperature_label']}**\n\n## {value['value']} °C")
                                    else:
                                        st.info(f"🌡️ **{name} {t['temperature_label']}**\n\n## {value['value']} °C")

                        with col2:
                            if fan_data:
                                st.markdown(f"### {t['fan_monitor_title']}")
                                for key, value in fan_data.items():
                                    name = key.split("/")[-1]
                                    fan_val = float(value['value'])

                                    if fan_val == 0:
                                        if "CPU" in key.upper():
                                            st.error(f"🌀 **{name} {t['fan_label']}**\n\n## {value['value']} RPM\n**❌ {t['cpu_fan_stopped']}**")
                                        else:
                                            st.warning(f"🌀 **{name} {t['fan_label']}**\n\n## {value['value']} RPM\n**⚠️ {t['fan_not_running']}**")
                                    else:
                                        st.success(f"🌀 **{name} {t['fan_label']}**\n\n## {value['value']} RPM")

                        st.markdown("<br>", unsafe_allow_html=True)

                        current_data = {k: v for k, v in hw_data.items() if "Current" in k}
                        case_data = {k: v for k, v in hw_data.items() if "Case Open" in k}
                        disk_data = {k: v for k, v in hw_data.items() if "DiskInfo" in k}

                        col1, col2 = st.columns(2)

                        with col1:
                            if disk_data:
                                st.markdown(f"### {t['disk_monitor_title']}")
                                for key, value in disk_data.items():
                                    disk_size_mb = float(value['value'])
                                    disk_size_gb = disk_size_mb / 1024

                                    if disk_size_gb > 1024:
                                        display_size = f"{disk_size_gb/1024:.1f} TB"
                                    else:
                                        display_size = f"{disk_size_gb:.1f} GB"

                                    st.info(f" **{t['total_disk_label']}**\n\n## {display_size}")

                        with col2:
                            st.empty()
                            # if current_data:
                            #     st.markdown(f"### {t['current_monitor_title']}")
                            #     for key, value in current_data.items():
                            #         name = key.split("/")[-1]
                            #         st.info(f"⚡ **{name}**\n\n## {value['value']} A")

            except Exception as e:
                st.error(f"❌ {t['susi_time_format_error']}{str(e)}")

    if auto_refresh_enabled:
        st.markdown("---")
        
        remaining_seconds = max(0, REFRESH_INTERVAL - seconds_since_last_update)
        progress = min(1.0, seconds_since_last_update / REFRESH_INTERVAL)
        
        st.progress(progress, text=f"⏰ {t['susi_next_refresh']} {int(remaining_seconds)} {t['susi_seconds']}")
        
        if remaining_seconds <= 0:
            new_data = load_susi_json()
            st.session_state.data = new_data
            st.session_state.last_update = NOW_TIMESTAMP
            st.session_state.update_counter += 1
            st.success("✅ 數據已自動更新！", icon="🔄")
            st.rerun() 
        
        if st.checkbox(t["debug_checkbox_label"], value=False):
            st.text(t["debug_current_time"].format(time=NOW_TIMESTAMP))
            st.text(t["debug_last_update"].format(time=st.session_state.last_update))
            st.text(t["debug_time_diff"].format(seconds=seconds_since_last_update))
            st.text(t["debug_remaining_time"].format(seconds=remaining_seconds))
            st.text(t["debug_progress"].format(progress=progress))
            
            if "system_time" in data:
                data_time = datetime.strptime(data["system_time"], "%Y-%m-%d %H:%M:%S")
                data_timestamp = data_time.timestamp()
                st.text(t["debug_data_timestamp"].format(timestamp=data_timestamp))
                st.text(t["debug_data_update_diff"].format(diff=abs(data_timestamp - st.session_state.last_update)))


elif page == t["sidebar_pages"][5]:
    st.subheader(t["analysis_title"])
    st.caption(t["analysis_yolo_caption"])

    log_path = "/home/amr/Desktop/robot_code/ui_status/yolo_full_log.json"

    try:
        with open(log_path, "r") as f:
            data = json.load(f)

        save_time_str = None
        for entry in reversed(data):
            if "保存時間" in entry:
                save_time_str = entry["保存時間"]
                break

        if save_time_str:
            from datetime import datetime, timedelta

            save_time = datetime.strptime(save_time_str, "%Y-%m-%d %H:%M:%S")
            now = datetime.now()
            diff = now - save_time

            if diff > timedelta(minutes=10):
                st.warning(t["analysis_yolo_time_expired"].format(time=save_time_str))
            else:
                all_objects = []
                for entry in data:
                    if "YOLO偵測結果" in entry:
                        for obj in entry["YOLO偵測結果"]:
                            obj_name = obj.get("物件")
                            if obj_name:
                                all_objects.append(obj_name)

                count = Counter(all_objects)
                df = pd.DataFrame({
                    "物件": list(count.keys()),
                    "次數": list(count.values())
                })

                color_chart = alt.Chart(df).mark_bar().encode(
                    x=alt.X("物件:N", sort='-y'),
                    y="次數:Q",
                    color="物件:N",
                    tooltip=["物件", "次數"]
                ).properties(
                    width=600,
                    height=500,
                )

                st.altair_chart(color_chart, use_container_width=True)
        else:
            st.warning(t["analysis_yolo_no_time"])

    except Exception as e:
        st.error(t["analysis_yolo_error"].format(error=str(e)))

    st.caption(t["analysis_path_caption"])
    csv_path = "/home/amr/Desktop/robot_code/picture_record/path_data_for_streamlit.csv"
    show_path_chart = True

    try:
        if not os.path.exists(csv_path):
            st.warning(t["analysis_path_no_csv"])
        else:
            df = pd.read_csv(csv_path)
            
            if len(df) > 0 and df.iloc[-1]['Real_X'] == "GeneratedTime":
                try:
                    from datetime import datetime, timedelta
                    last_save_time = df.iloc[-1]['Real_Y']
                    save_time = datetime.strptime(str(last_save_time), "%Y-%m-%d %H:%M:%S")
                    now = datetime.now()
                    time_diff = now - save_time

                    if time_diff > timedelta(minutes=10):
                        st.warning(t["analysis_path_time_expired"].format(time=last_save_time))
                        # st.stop()
                        show_path_chart = False
                    
                    df = df.iloc[:-1]

                except Exception as e:
                    st.warning(f"⚠️ 時間格式錯誤，略過時間比對：{e}")
                
            if show_path_chart:

                path_df = df
                
                real_data = path_df[['Real_X', 'Real_Y']].dropna()
                plan_data = path_df[['Plan_X', 'Plan_Y']].dropna()
                
                if not plan_data.empty and 'start_pose' in st.session_state and st.session_state.start_pose:
                    start_x = st.session_state.start_pose['x']
                    start_y = st.session_state.start_pose['y']
                    
                    start_point = pd.DataFrame({'Plan_X': [start_x], 'Plan_Y': [start_y]})
                    plan_data = pd.concat([start_point, plan_data], ignore_index=True)
                
                if real_data.empty and plan_data.empty:
                    st.warning(t["analysis_path_empty"])
                else:
                    fig = go.Figure()
                    if not real_data.empty:
                        fig.add_trace(go.Scatter(
                            x=real_data['Real_X'],
                            y=real_data['Real_Y'],
                            mode='lines+markers',
                            line=dict(color='green'),
                            marker=dict(symbol='circle', size=6),
                            name='Real Route'
                        ))
                    if not plan_data.empty:
                        fig.add_trace(go.Scatter(
                            x=plan_data['Plan_X'],
                            y=plan_data['Plan_Y'],
                            mode='lines+markers',
                            line=dict(color='#FFA500', dash='dash'),
                            marker=dict(symbol='x', size=8),
                            name='Plan Route'
                        ))
                    fig.update_layout(
                        title='Robot Navigation Path',
                        xaxis_title='X',
                        yaxis_title='Y',
                        showlegend=True,
                        xaxis=dict(showgrid=True),
                        yaxis=dict(showgrid=True, scaleanchor="x", scaleratio=1),
                        width=800,
                        height=600
                    )
                    st.plotly_chart(fig, use_container_width=True)
                
    except Exception as e:
        st.error(t["analysis_time_error"].format(error=str(e)))

    st.caption(t["analysis_object_caption"])
    try:
        with open(log_path, "r") as f:
            data = json.load(f)

        save_time_str = None
        for entry in reversed(data):
            if "保存時間" in entry:
                save_time_str = entry["保存時間"]
                break

        if save_time_str:
            from datetime import datetime, timedelta
            save_time = datetime.strptime(save_time_str, "%Y-%m-%d %H:%M:%S")
            now = datetime.now()
            diff = now - save_time

            if diff > timedelta(minutes=10):
                st.warning(t["analysis_object_time_expired"].format(time=save_time_str))
            else:

                records = []
                for entry in data:
                    if "YOLO偵測結果" in entry:
                        for obj in entry["YOLO偵測結果"]:
                            if "x" in obj and "y" in obj:
                                records.append({
                                    "物件": obj["物件"],
                                    "x": obj["x"],
                                    "y": obj["y"],
                                    "信心分數": obj.get("信心分數", 0.5)
                                })

                df = pd.DataFrame(records)
                
                if df.empty:
                    st.warning(t["analysis_object_no_data"])
                else:
                    df["原始信心分數"] = df["信心分數"] 
                    df["信心分數_normalized"] = df["原始信心分數"].clip(0.01, 1.0)
                    conf_min = df["信心分數_normalized"].min()
                    conf_max = df["信心分數_normalized"].max()
                    
                    if conf_max != conf_min:
                        df["信心分數_normalized"] = (df["信心分數_normalized"] - conf_min) / (conf_max - conf_min) ** 10
                    else:
                        df["信心分數_normalized"] = 1.0
                    
                    df["size_for_plot"] = df["信心分數_normalized"] * 150
                    fig = px.scatter(
                        df,
                        x="x",
                        y="y",
                        color="物件",
                        size="size_for_plot",
                        hover_data=["物件", "x", "y", "原始信心分數"],  
                        size_max=15,
                        opacity=0.6,
                        title=t["analysis_object_title"]
                    )
                    fig.update_layout(
                        yaxis=dict(scaleanchor="x", scaleratio=1),  
                        width=800,
                        height=600
                    )
                    st.plotly_chart(fig, use_container_width=True)
        else:
            st.warning(t["analysis_yolo_no_time"])

    except Exception as e:
        st.error(t["analysis_yolo_error"].format(error=str(e)))


elif page == t["sidebar_pages"][6]:

    with st.expander(t["power_title"]):
        st.caption(t["power_caption"])
        if st.button(t["power_button"]):
            st.warning(t["power_warning"])
            try:
                subprocess.run(["sudo", "poweroff"])
            except Exception as e:
                st.error(t["power_error"] + str(e))

    with st.expander(t["reboot_title"]):
        st.caption(t["reboot_caption"])
        if st.button(t["reboot_button"]):
            st.warning(t["reboot_warning"])
            try:
                subprocess.run(["sudo", "reboot"])
            except Exception as e:
                st.error(t["reboot_error"] + str(e))


    with st.expander(t["language_title"]):
        st.caption(t["language_caption"])
        language_options = ["繁體中文", "日本語", "한국어", "English"]

        language = st.radio(
            t["language_radio"],
            options=language_options,
            index=language_options.index(st.session_state.language)
            if st.session_state.language in language_options else 0
        )

        if language != st.session_state.language:
            st.session_state.language = language
            st.rerun()

        st.success(t["language_success"] + language)


    with st.expander(t["theme_title"]):
        st.caption(t["theme_caption"])
        if "theme" not in st.session_state:
            st.session_state.theme = "深色"
        theme = st.radio(
            t["theme_radio"],
            options=["深色", "淺色"],
            index=0 if st.session_state.theme == "深色" else 1
        )
        st.session_state.theme = theme
        st.success(t["theme_success"] + theme)


    with st.expander(t["contact_title"]):
        st.caption(t["contact_caption"])
        col1, col2, col3 = st.columns(3)

        with col1:
            st.markdown("""
            <div style="border: 1px solid #ccc; border-radius: 10px; padding: 15px; background-color: #1e1e1e; line-height: 2.0;">
                <h4>👨‍💼 Steve Chang</h4>
                <p>
                    💼  AVP（副總經理）<br>
                    🏷️ ACL_Embedded_Embedded Sector<br>
                    📧 <a href="mailto:Steve.Chang@advantech.com.tw" style="color: #4EA8DE;">Steve.Chang@advantech.com.tw</a><br>
                    ☎️ VOIP: 511 EXT: 9279<br>
                    🏢 Advantech ACL<br>
                    📍 台灣桃園市龜山區樂善里文德路27-3號
                </p>
            </div>
            """, unsafe_allow_html=True)

        with col2:
            st.markdown("""
            <div style="border: 1px solid #555; border-radius: 12px; padding: 15px;
                        background-color: #1e1e1e; color: #eee;
                        box-shadow: 2px 2px 5px #000; line-height: 2.0;">
                <h4>🧑‍💼 Jack Tsao</h4>
                <p>
                    💼 Director（協理）<br>
                    🏷️ FAE<br>
                    📧 <a href="mailto:Jack.Tsao@advantech.com" style="color: #4EA8DE;">Jack.Tsao@advantech.com.tw</a><br>
                    ☎️ VOIP:516 EXT:4260<br>
                    🏢 Advantech AJP<br>
                    📍 日本東京都台東区浅草6-16-3
                </p>
            </div>
            """, unsafe_allow_html=True)


        with col3:
            st.markdown("""
            <div style="border: 1px solid #555; border-radius: 12px; padding: 15px;
                        background-color: #1e1e1e; color: #eee;
                        box-shadow: 2px 2px 5px #000; line-height: 2.0;">
                <h4>👨‍💻 Ray Zheng</h4>
                <p>
                    🧪 Lv1 Engineer<br />
                    🏷️ ACL_Embedded_Linux Service<br />
                    📧 <a href="mailto:Ray.Zheng@advantech.com.tw" style="color: #4EA8DE;">Ray.Zheng@advantech.com.tw</a><br />
                    ☎️ VOIP:511 EXT:9490<br />
                    🏢 Advantech ACL<br>
                    📍 台灣桃園市龜山區樂善里文德路27-3號
                </p>
            </div>
            """, unsafe_allow_html=True)
    
    with st.expander(t["logout_title"]):
        st.caption(t["logout_caption"])
        if st.button(t["logout_button"]):
            for key in list(st.session_state.keys()):
                del st.session_state[key]
            st.success(t["logout_success"])










st.markdown("""
    <style>
        /* 每個 radio label 做為區塊顯示並增加下邊距 */
        [data-testid="stSidebar"] div[role="radiogroup"] > label {
            display: flex;
            align-items: center;       /* ✅ 垂直置中：對齊文字與圓圈 */
            margin-bottom: 10px;       /* ✅ 行距 10px */
            gap: 0.5rem;               /* ✅ 文字與圓圈之間間距（可選） */
        }

        /* 被選中項目可加強顯示 */
        [data-testid="stSidebar"] div[role="radiogroup"] > label[data-selected="true"] {
            font-weight: bold;
        }
    </style>
""", unsafe_allow_html=True)

st.markdown("""
    <style>
    /* 改變 expander 摺疊區標題的背景色與字體樣式 */
    details > summary {
        background-color: #1e4438 !important;
        color: white !important;
        padding: 12px;
        border-radius: 5px;
        font-size: 16px;
        font-weight: 600;
        list-style: none;
    }

    /* 移除 summary 前面的 ▸ 符號 */
    details > summary::-webkit-details-marker {
        display: none;
    }
    </style>
    """, unsafe_allow_html=True)
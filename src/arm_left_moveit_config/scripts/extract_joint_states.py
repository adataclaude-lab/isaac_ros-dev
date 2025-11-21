#!/usr/bin/env python3
"""
從 ROS2 Bag 檔案中提取 joint_states 資料
並輸出為 CSV 格式供分析使用
"""

import argparse
import csv
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
from pathlib import Path


def extract_joint_states_from_bag(bag_path, output_csv):
    """
    從 bag 檔案提取 joint_states
    
    Args:
        bag_path: rosbag 目錄路徑
        output_csv: 輸出 CSV 檔案路徑
    """
    
    # 開啟 bag 資料庫
    db_path = Path(bag_path) / 'rosbag2_2025_11_19-17_01_57_0.db3'
    
    if not db_path.exists():
        # 尋找第一個 .db3 檔案
        db_files = list(Path(bag_path).glob('*.db3'))
        if not db_files:
            print(f"錯誤: 在 {bag_path} 中找不到 .db3 檔案")
            return
        db_path = db_files[0]
    
    print(f"讀取 bag 檔案: {db_path}")
    
    conn = sqlite3.connect(str(db_path))
    cursor = conn.cursor()
    
    # 取得 topic 資訊
    cursor.execute("SELECT id, name, type FROM topics WHERE name='/joint_states'")
    topic_info = cursor.fetchone()
    
    if not topic_info:
        print("錯誤: 找不到 /joint_states topic")
        conn.close()
        return
    
    topic_id, topic_name, topic_type = topic_info
    print(f"找到 topic: {topic_name} (type: {topic_type})")
    
    # 取得訊息
    cursor.execute("""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id = ?
        ORDER BY timestamp
    """, (topic_id,))
    
    messages = cursor.fetchall()
    print(f"找到 {len(messages)} 筆訊息")
    
    # 準備 CSV 輸出
    msg_type = get_message(topic_type)
    
    with open(output_csv, 'w', newline='') as csvfile:
        writer = None
        
        for idx, (timestamp, data) in enumerate(messages):
            # 反序列化訊息
            msg = deserialize_message(data, msg_type)
            
            # 第一筆訊息時建立 CSV header
            if writer is None:
                fieldnames = ['timestamp', 'time_sec'] + list(msg.name)
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
            
            # 寫入資料
            row = {
                'timestamp': timestamp,
                'time_sec': timestamp / 1e9,  # 轉換為秒
            }
            
            for joint_name, position in zip(msg.name, msg.position):
                row[joint_name] = position
            
            writer.writerow(row)
            
            # 顯示進度
            if (idx + 1) % 100 == 0 or idx == len(messages) - 1:
                print(f"處理進度: {idx + 1}/{len(messages)}")
    
    conn.close()
    print(f"\n完成! 資料已輸出到: {output_csv}")


def print_bag_info(bag_path):
    """顯示 bag 檔案資訊"""
    
    db_files = list(Path(bag_path).glob('*.db3'))
    if not db_files:
        print(f"錯誤: 在 {bag_path} 中找不到 .db3 檔案")
        return
    
    db_path = db_files[0]
    conn = sqlite3.connect(str(db_path))
    cursor = conn.cursor()
    
    # 取得所有 topics
    cursor.execute("SELECT name, type, COUNT(*) as msg_count FROM topics JOIN messages ON topics.id = messages.topic_id GROUP BY topics.id")
    topics = cursor.fetchall()
    
    print("\n=== Bag 檔案資訊 ===")
    print(f"路徑: {bag_path}")
    print(f"資料庫: {db_path.name}")
    print("\nTopics:")
    print(f"{'Topic':<40} {'Type':<40} {'訊息數':<10}")
    print("-" * 90)
    
    for topic_name, topic_type, msg_count in topics:
        print(f"{topic_name:<40} {topic_type:<40} {msg_count:<10}")
    
    # 取得時間範圍
    cursor.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages")
    min_time, max_time = cursor.fetchone()
    
    duration = (max_time - min_time) / 1e9  # 轉換為秒
    
    print(f"\n時間範圍:")
    print(f"  開始: {min_time / 1e9:.3f} 秒")
    print(f"  結束: {max_time / 1e9:.3f} 秒")
    print(f"  總長: {duration:.3f} 秒")
    
    conn.close()


def main():
    parser = argparse.ArgumentParser(
        description='從 ROS2 bag 檔案中提取 joint_states 資料'
    )
    
    parser.add_argument(
        'bag_path',
        help='Rosbag 目錄路徑'
    )
    
    parser.add_argument(
        '-o', '--output',
        default='joint_states.csv',
        help='輸出 CSV 檔案名稱 (預設: joint_states.csv)'
    )
    
    parser.add_argument(
        '-i', '--info',
        action='store_true',
        help='只顯示 bag 檔案資訊,不提取資料'
    )
    
    args = parser.parse_args()
    
    if args.info:
        print_bag_info(args.bag_path)
    else:
        extract_joint_states_from_bag(args.bag_path, args.output)


if __name__ == '__main__':
    main()

import serial
import numpy as np
import time
import pandas as pd

# シリアルポートの設定
# COMポートはシステムによって異なるので適宜変更してください。
# 115200はArduino側のbaudrateと一致させる必要があります。
ser = serial.Serial('COM4', 115200, timeout=1)

# シリアルをフラッシュ
ser.flush()

unit = 1000000000

# Q学習アルゴリズムのパラメータ（例）
learning_rate = 0.1
discount_factor = 0.95
exploration_rate = 0.1

# 状態空間と行動空間の設定
num_states = 400 # 状態の数
num_actions = 2  # 行動の数（例：0=左、1=右）
Q = np.zeros((num_states, num_actions))  # Qテーブルの初期化

def waitUntilCycleTime(CYCLE_TIME, initial_time, n, t):
    # Wait until the sampling cycle is exceeded
    while t < n*CYCLE_TIME:
        t = (time.perf_counter_ns() - initial_time) / unit #Diff from initial time
    n+=1
    return n, t

def get_state_from_sensor_data(sensor_data):
    # センサーデータから状態を決定するロジック
    # ここではシンプルに角度を基に離散化された状態を返すとします。
    # 実際にはセンサーデータに応じて適切に処理を行う必要があります。
    angle = sensor_data['angle']
    state = min(int(angle // (360 / num_states)), num_states - 1)
    return state

def choose_action(state):
    # ε-greedy法で行動を選択
    if np.random.rand() < exploration_rate:
        return np.random.choice(num_actions)
    else:
        return np.argmax(Q[state])
    
    

def update_Q_table(state, action, reward, next_state):
    # Qテーブルの更新
    best_next_action = np.argmax(Q[next_state])
    td_target = reward + discount_factor * Q[next_state][best_next_action]
    td_error = td_target - Q[state][action]
    Q[state][action] += learning_rate * td_error

def send_action_to_arduino(action):
    # 選択された行動をArduinoに送信
    ser.write(f"{action}\n".encode())

def reset_episode():
    # Arduinoにリセットコマンドを送信
    ser.write(b"-1\n")

def reward_function(angle):
    # 報酬関数の設定
    reward = - np.cos(np.abs(angle) * np.pi / 200) *100
    reward = int(reward)
    return reward

def main():
    try:
        while True:
            while True:
                # Calibrationの終了を待つ
                if ser.in_waiting > 0:
                    response = ser.readline().decode().strip()
                    if response == "ce":
                        print("CartPoal is calibrated.")
                        break
            
            time.sleep(2) #Arduinoの初期化待ち
    
            n = 1
            t = 0

            sensor_data_raw = ser.readline().decode().strip()
            _, _, init_data_number, _, _ = map(int, sensor_data_raw.split(','))
            # エピソード開始時刻の記録
            episode_start_time = time.perf_counter_ns()
            data_number = init_data_number

            Data_log = pd.DataFrame(columns=['Data_Number', 'State', 'Action', 'Reward', 'Angle', 'Acceleration', 'pos_cart'])

            while data_number-init_data_number < 150:  # 15秒間のエピソード
                if ser.in_waiting > 0:
                    sensor_data_raw = ser.readline().decode().strip()
                    if sensor_data_raw:  # データがある場合
                        pos_cart, state_cart, data_number, angle, acceleration = map(int, sensor_data_raw.split(','))
                        sensor_data = {'pos_cart' : pos_cart,'state_cart': state_cart, 'data_number': data_number, 'angle': angle, 'acceleration': acceleration}
                        
                        if sensor_data['state_cart'] == 1:
                            print("カートが範囲外に出ました。リセットします。")
                            break
                        state = get_state_from_sensor_data(sensor_data)
                        action = choose_action(state)
                        
                        send_action_to_arduino(action)

                        #reward = 1 if  (190 < np.abs(angle) and np.abs(angle) < 210) else -1
                        reward = reward_function(angle)
                        next_state = get_state_from_sensor_data(sensor_data)
                        update_Q_table(state, action, reward, next_state)

                # sampling cycle time
                #n, t = waitUntilCycleTime(0.1, episode_start_time, n, t)
                print("Data_Number", data_number, "State:", state, "Action:", action,"Reward:", reward, "Angle:", angle, "Acceleration:", acceleration, "pos_cart:", pos_cart)
                #printしたものをdataframeに保存
                df = pd.DataFrame([[data_number, state, action, reward, angle, acceleration, pos_cart]], columns=['Data_Number', 'State', 'Action', 'Reward', 'Angle', 'Acceleration', 'pos_cart'])
                Data_log = pd.concat([Data_log, df], axis=0)

            print("エピソード終了: 15秒経過")

            # Data_logをcsvファイルに保存
            #現在時刻をファイル名にする
            now = time.strftime('%Y%m%d_%H%M%S')
            Data_log.to_csv(f'action/{now}.csv', index=False)

        
            reset_episode()  # エピソードのリセット

    except KeyboardInterrupt:
        print("終了します。")
        ser.close()

if __name__ == '__main__':
    main()

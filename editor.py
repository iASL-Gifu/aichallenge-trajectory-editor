import csv
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backend_bases import MouseButton
import tkinter as tk
from tkinter import filedialog
import subprocess



class PlotTool:
    def __init__(self, master):
        self.master = master
        self.master.title("CSV Plot Tool")

        # フレームを作成してウィジェットを配置
        self.frame = tk.Frame(master)
        self.frame.pack()

        # CSVファイルのロードボタン
        self.load_button = tk.Button(self.frame, text="Load trj", command=self.load_csv)
        self.load_button.grid(row=0, column=0, padx=5, pady=5)

        # Mapのロードボタン(csvファイルを読み込んで表示)
        self.load_map_button = tk.Button(self.frame, text="Load lane", command=self.load_map)
        self.load_map_button.grid(row=0, column=1, padx=5, pady=5)

        # ラベル表示のチェックボタン
        self.show_labels_var = tk.BooleanVar(value=False)  # ラベルの表示・非表示を管理する変数
        self.show_labels_checkbutton = tk.Checkbutton(self.frame, text="Show Labels", variable=self.show_labels_var, command=self.plot_data)
        self.show_labels_checkbutton.grid(row=0, column=2, padx=5, pady=5)

        # 点追加のチェックボタン
        self.add_point_var = tk.BooleanVar(value=False)  # 点追加のモードを管理する変数
        self.add_point_checkbutton = tk.Checkbutton(self.frame, text="Add Point", variable=self.add_point_var, command=self.set_add_point)
        self.add_point_checkbutton.grid(row=0, column=3, padx=5, pady=5)

        # 点移動のチェックボタン
        self.move_point_var = tk.BooleanVar(value=False)
        self.move_point_checkbutton = tk.Checkbutton(self.frame, text="Move Point", variable=self.move_point_var, command=self.set_move_point)
        self.move_point_checkbutton.grid(row=0, column=4, padx=5, pady=5)

        # 点削除のチェックボタン
        self.delete_point_var = tk.BooleanVar(value=False)
        self.delete_point_checkbutton = tk.Checkbutton(self.frame, text="Delete Point", variable=self.delete_point_var, command=self.set_delete_point)
        self.delete_point_checkbutton.grid(row=0, column=5, padx=5, pady=5)

        # ラベル編集のチェックボタン
        self.edit_label_var = tk.BooleanVar(value=False)
        self.edit_label_checkbutton = tk.Checkbutton(self.frame, text="Edit Label", variable=self.edit_label_var, command=self.set_edit_label)
        self.edit_label_checkbutton.grid(row=0, column=6, padx=5, pady=5)

        # CSVファイルの保存ボタン
        self.save_button = tk.Button(self.frame, text="Save CSV", command=self.save_csv)
        self.save_button.grid(row=0, column=7, padx=5, pady=5)

        # 実行ボタン
        self.run_button = tk.Button(self.frame, text="Post", command=self.run)
        self.run_button.grid(row=0, column=8, padx=5, pady=5)

        # 終了ボタン
        self.quit_button = tk.Button(self.frame, text="Quit", command=self.master.quit)
        self.quit_button.grid(row=0, column=9, padx=5, pady=5)

        # Matplotlibの図と軸を設定
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.patch.set_facecolor('white')  # 図の背景を白色に設定
        self.ax.set_facecolor('white')         # 軸の背景を白色に設定

        # Matplotlibの図をTkinterに埋め込む
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack()

        # イベントハンドラの設定
        self.cid_press = self.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_release = self.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.cid_scroll = self.canvas.mpl_connect('scroll_event', self.on_scroll)  # スクロールイベント

        self.x, self.y, self.labels = [], [], []
        self.z, self.x_q, self.y_q, self.z_q, self.w_q = [], [], [], [], []
        self.inner_map_x, self.inner_map_y = [], []
        self.outer_map_x, self.outer_map_y = [], []
        self.texts = []  # ラベル表示用のテキスト
        self.header = []  # CSVファイルのヘッダー
        self.last_path = None  # 最後に選択したファイルのパス
        self.selected_point = None
        self.selected_line = None
        self.active_label = None  # アクティブなラベルの保持

        # ズーム倍率の初期化
        self.zoom_scale = 1.1
        self.pan_active = False   # パン（画面移動）状態のフラグ
        self.pan_start = None     # パンの開始位置を記録する

    def set_add_point(self):
        self.move_point_var.set(False)
        self.edit_label_var.set(False)
        self.delete_point_var.set(False)

    def set_move_point(self):
        self.add_point_var.set(False)
        self.edit_label_var.set(False)
        self.delete_point_var.set(False)

    def set_edit_label(self):
        self.add_point_var.set(False)
        self.move_point_var.set(False)
        self.delete_point_var.set(False)

    def set_delete_point(self):
        self.add_point_var.set(False)
        self.move_point_var.set(False)
        self.edit_label_var.set(False)

    def load_csv(self):
        file_path = filedialog.askopenfilename()
        if not file_path:
            return
        self.x, self.y, self.labels = [], [], []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            i = 0
            for row in reader:
                if i == 0:
                    i += 1
                    self.header = row
                else:
                    self.x.append(float(row[0]))
                    self.y.append(float(row[1]))
                    self.z.append(float(row[2]))
                    self.x_q.append(float(row[3]))
                    self.y_q.append(float(row[4]))
                    self.z_q.append(float(row[5]))
                    self.w_q.append(float(row[6]))
                    self.labels.append(float(row[7]))
            print(len(self.x))
        
        # 軸の範囲をデータに合わせて調整
        self.ax.set_xlim(min(self.x) - 5, max(self.x) + 5)
        self.ax.set_ylim(min(self.y) - 5, max(self.y) + 5)
        
        self.plot_data()
    
    def run(self):
        # example.shを実行するPythonスクリプト
        path = './.post/post.csv'
        self.save_csv(path)
        # pathを絶対パスに変換
        path = os.path.abspath(path)
        try:
            result = subprocess.run(['bash', 'shell.sh', path], check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e.stderr}")

    def load_map(self):
        file_path = filedialog.askopenfilename()
        if not file_path:
            return
        self.inner_map_x, self.inner_map_y, self.outer_map_x, self.outer_map_y = [], [], [], []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if row[0]:
                    self.inner_map_x.append(float(row[0]))
                    self.inner_map_y.append(float(row[1]))
                if row[2]:
                    self.outer_map_x.append(float(row[2]))
                    self.outer_map_y.append(float(row[3]))
        
        self.ax.plot(self.inner_map_x, self.inner_map_y, 'b-')
        self.ax.plot(self.outer_map_x, self.outer_map_y, 'b-')
        self.canvas.draw()

    def save_csv(self, path=None):
        self.calc_quaternion()
        if path is None:
            file_path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
        else:
            file_path = path
        if not file_path:
            return
        self.last_path = file_path
        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.header)
            for i in range(len(self.x)):
                writer.writerow([self.x[i], self.y[i], self.z[i], self.x_q[i], self.y_q[i], self.z_q[i], self.w_q[i], self.labels[i]])

    def plot_data(self):
        # 現在のxlimとylimを保存
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        self.ax.clear()  # 現在のプロットをクリア
        self.ax.set_facecolor('white')  # 軸の背景を白色で維持
        self.ax.plot(self.x, self.y, 'o', picker=5)  # 点をプロット
        self.ax.plot(self.x, self.y, 'g-')            # 点を線で接続
        self.ax.plot(self.inner_map_x, self.inner_map_y, 'b-')
        self.ax.plot(self.outer_map_x, self.outer_map_y, 'b-')

        # 既存のテキスト（ラベル）を削除
        for text in self.texts:
            text.remove()
        self.texts = []

        # チェックボタンの状態に応じてラベルを表示
        if self.show_labels_var.get():
            for i in range(len(self.x)):
                txt = self.ax.text(self.x[i], self.y[i], self.labels[i], fontsize=12, ha='right', color='blue')
                self.texts.append(txt)

        # 保存していたxlimとylimを再設定
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)

        self.canvas.draw()  # 描画を更新

    def on_click(self, event):
        if event.inaxes != self.ax: return
        if event.button == MouseButton.LEFT:
            if self.move_point_var.get():
                self.selected_point = self.find_nearest_point(event.xdata, event.ydata)
            elif self.add_point_var.get():
                self.selected_line = self.find_nearest_line(event.xdata, event.ydata)
            elif self.delete_point_var.get():
                point_idx = self.find_nearest_point(event.xdata, event.ydata)
                if point_idx is not None:
                    self.delete_point(point_idx)
            elif self.edit_label_var.get():
                point_idx = self.find_nearest_point(event.xdata, event.ydata)
                if point_idx is not None:
                    self.active_label = point_idx  # アクティブなラベルを設定
                    self.edit_label(point_idx)     # ラベル編集
                    self.plot_data()               # ラベルを表示
        elif event.button == MouseButton.RIGHT:
            self.selected_line = None
            # 画面移動のために右クリックを使用
            self.pan_active = True
            self.pan_start = (event.xdata, event.ydata)

    def on_release(self, event):
        self.selected_point = None
        self.selected_line = None
        self.pan_active = False  # 画面移動を終了

    def on_motion(self, event):
        if self.selected_point is not None and event.inaxes == self.ax:
            # 点をドラッグして移動
            self.x[self.selected_point] = event.xdata
            self.y[self.selected_point] = event.ydata
            self.plot_data()
        elif self.pan_active and event.inaxes == self.ax:
            # 画面移動（パン）機能
            dx = self.pan_start[0] - event.xdata
            dy = self.pan_start[1] - event.ydata

            # 現在の軸範囲を取得して、ドラッグ量に基づいて新しい範囲を設定
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()

            self.ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
            self.ax.set_ylim(ylim[0] + dy, ylim[1] + dy)

            self.canvas.draw()  # 描画を更新

    def on_scroll(self, event):
        # スクロールイベントでズームイン/ズームアウト
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        # ズームの中心はマウスの位置
        xdata, ydata = event.xdata, event.ydata

        if event.button == 'up':  # ズームイン
            scale_factor = self.zoom_scale
        elif event.button == 'down':  # ズームアウト
            scale_factor = 1 / self.zoom_scale
        else:
            return

        # 新しい範囲を計算
        new_xlim = [(x - xdata) * scale_factor + xdata for x in xlim]
        new_ylim = [(y - ydata) * scale_factor + ydata for y in ylim]

        # 新しい範囲をセット
        self.ax.set_xlim(new_xlim)
        self.ax.set_ylim(new_ylim)

        # 再描画
        self.canvas.draw()

    def find_nearest_point(self, x, y):
        distances = np.hypot(np.array(self.x) - x, np.array(self.y) - y)
        min_idx = np.argmin(distances)
        if distances[min_idx] < 1:
            return min_idx
        return None

    def find_nearest_line(self, x, y):
        min_distance = float('inf')
        nearest_idx = None
        nearest_point = None

        for i in range(len(self.x) - 1):
            x0, y0 = self.x[i], self.y[i]
            x1, y1 = self.x[i + 1], self.y[i + 1]
            px, py = self.project_point_on_line(x0, y0, x1, y1, x, y)
            distance = np.hypot(px - x, py - y)
            
            if distance < min_distance:
                min_distance = distance
                nearest_idx = i
                nearest_point = (px, py)

        if nearest_point is not None:
            # 線分上の最近接点に新しい点を追加
            self.x.insert(nearest_idx + 1, nearest_point[0])
            self.y.insert(nearest_idx + 1, nearest_point[1])
            self.labels.insert(nearest_idx + 1, 0.0)
            self.plot_data()
            return nearest_idx + 1
        return None
    
    def project_point_on_line(self, x0, y0, x1, y1, x, y):
        dx, dy = x1 - x0, y1 - y0
        if dx == 0 and dy == 0:
            return x0, y0
        t = ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)
        t = np.clip(t, 0, 1)
        px, py = x0 + t * dx, y0 + t * dy
        return px, py
    


    def is_near_line(self, x0, y0, x1, y1, x, y, tol=0.1):
        d = np.abs((y1 - y0) * x - (x1 - x0) * y + x1 * y0 - y1 * x0) / np.hypot(x1 - x0, y1 - y0)
        return d < tol

    def delete_point(self, point_idx):
        self.x.pop(point_idx)
        self.y.pop(point_idx)
        self.labels.pop(point_idx)
        self.plot_data()

    def edit_label(self, point_idx):
        # ポイントのラベルをダブルクリックで編集
        new_label = tk.simpledialog.askstring("Edit Label", f"Edit label for point {point_idx}", initialvalue=self.labels[point_idx])
        if new_label is not None:
            self.labels[point_idx] = new_label
            self.plot_data()

    def calc_quaternion(self):
        # すべての点において、次の点を使ってクォータニオンを計算(yawのみ)
        self.x_q.clear()
        self.y_q.clear()
        self.z_q.clear()
        self.w_q.clear()
        for i in range(len(self.x) - 1):
            x0, y0, z0 = self.x[i], self.y[i], self.z[i]
            x1, y1, z1 = self.x[i + 1], self.y[i + 1], self.z[i + 1]
            dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
            yaw = np.arctan2(dy, dx)
            q = self.quaternion_from_euler(0, 0, yaw)
            self.x_q.append(q[0])
            self.y_q.append(q[1])
            self.z_q.append(q[2])
            self.w_q.append(q[3])
        x0, y0, z0 = self.x[-1], self.y[-1], self.z[-1]
        x1, y1, z1 = self.x[0], self.y[0], self.z[0]
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        yaw = np.arctan2(dy, dx)
        q = self.quaternion_from_euler(0, 0, yaw)
        self.x_q.append(q[0])
        self.y_q.append(q[1])
        self.z_q.append(q[2])
        self.w_q.append(q[3])


    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q


if __name__ == "__main__":
    root = tk.Tk()
    plot_tool = PlotTool(root)
    root.mainloop()

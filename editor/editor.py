import csv
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backend_bases import MouseButton
from matplotlib.collections import LineCollection
import tkinter as tk
from tkinter import filedialog, simpledialog
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
        self.edit_label_checkbutton = tk.Checkbutton(self.frame, text="Edit a Label", variable=self.edit_label_var, command=self.set_edit_label)
        self.edit_label_checkbutton.grid(row=0, column=6, padx=5, pady=5)

        # ラベル一括編集のチェックボタン
        self.calculate_speed_var = tk.BooleanVar(value=False)
        self.calculate_speed_checkbutton = tk.Checkbutton(self.frame, text="Edit Labels", variable=self.calculate_speed_var, command=self.set_edit_labels)
        self.calculate_speed_checkbutton.grid(row=0, column=7, padx=5, pady=5)

        # CSVファイルの保存ボタン
        self.save_button = tk.Button(self.frame, text="Save CSV", command=self.save_csv)
        self.save_button.grid(row=0, column=8, padx=5, pady=5)

        # 実行ボタン
        self.run_button = tk.Button(self.frame, text="Post", command=self.run)
        self.run_button.grid(row=0, column=9, padx=5, pady=5)

        # 終了ボタン
        self.quit_button = tk.Button(self.frame, text="Quit", command=self.master.quit)
        self.quit_button.grid(row=0, column=10, padx=5, pady=5)

        # ダークモードのチェックボックス
        self.dark_mode_var = tk.BooleanVar(value=False)  # ダークモードを管理する変数
        self.dark_mode_checkbutton = tk.Checkbutton(self.frame, text="Dark Mode", variable=self.dark_mode_var, command=self.toggle_dark_mode)
        self.dark_mode_checkbutton.grid(row=0, column=11, padx=5, pady=5)

        self.move_selected_var = tk.BooleanVar(value=False)
        self.move_selected_checkbutton = tk.Checkbutton(self.frame, text="Move Selected Points", variable=self.move_selected_var, command=self.set_move_selected)
        self.move_selected_checkbutton.grid(row=0, column=12, padx=5, pady=5)

        self.straight_line_var = tk.BooleanVar(value=False)
        self.straight_line_checkbutton = tk.Checkbutton(self.frame, text="Straight Line", variable=self.straight_line_var, command=self.set_straight_line)
        self.straight_line_checkbutton.grid(row=0, column=13, padx=5, pady=5)

        self.undo_button = tk.Button(self.frame, text="Undo", command=self.undo)
        self.undo_button.grid(row=0, column=14, padx=5, pady=5)
        self.redo_button = tk.Button(self.frame, text="Redo", command=self.redo)
        self.redo_button.grid(row=0, column=15, padx=5, pady=5)

        # オプションメニュー
        self.options_frame = tk.Frame(master)
        self.options_frame.pack()

        # 初期ラベル値の設定
        self.initial_label_value = tk.DoubleVar(value=0.0)
        tk.Label(self.options_frame, text="Initial Label Value:").grid(row=0, column=0, padx=5, pady=5)
        self.initial_label_entry = tk.Entry(self.options_frame, textvariable=self.initial_label_value)
        self.initial_label_entry.grid(row=0, column=1, padx=5, pady=5)
        self.reset_button = tk.Button(self.options_frame, text="reset view", command=self.reset_view)
        self.reset_button.grid(row=0, column=2, padx=5, pady=5)

        # ラベル値の加減するための設定
        self.add_label_value = tk.DoubleVar(value=0.0)
        tk.Label(self.options_frame, text="+/- Value:").grid(row=0, column=3, padx=5, pady=5)
        self.add_label_entry = tk.Entry(self.options_frame, textvariable=self.add_label_value)
        self.add_label_entry.grid(row=0, column=4, padx=5, pady=5)
        self.add_button = tk.Button(self.options_frame, text="+", command=self.add_label)
        self.add_button.grid(row=0, column=5, padx=5, pady=5)
        self.sub_button = tk.Button(self.options_frame, text="-", command=self.sub_label)
        self.sub_button.grid(row=0, column=6, padx=5, pady=5)

        # 色変更オフセット値の設定
        self.low_offset_value = tk.DoubleVar(value=10.0)
        tk.Label(self.options_frame, text="Low Color Value:").grid(row=1, column=0, padx=5, pady=5)
        self.color_offset_entry = tk.Entry(self.options_frame, textvariable=self.low_offset_value)
        self.color_offset_entry.grid(row=1, column=1, padx=5, pady=5)

        self.high_offset_value = tk.DoubleVar(value=30.0)
        tk.Label(self.options_frame, text="High Color Value:").grid(row=1, column=2, padx=5, pady=5)
        self.color_offset_entry = tk.Entry(self.options_frame, textvariable=self.high_offset_value)
        self.color_offset_entry.grid(row=1, column=3, padx=5, pady=5)

        # ラベル値に一定割合かける処理
        self.multiply_label_value = tk.DoubleVar(value=1.0)
        tk.Label(self.options_frame, text="Multiply Value:").grid(row=1, column=4, padx=5, pady=5)
        self.multiply_label_entry = tk.Entry(self.options_frame, textvariable=self.multiply_label_value)
        self.multiply_label_entry.grid(row=1, column=5, padx=5, pady=5)
        self.multiply_button = tk.Button(self.options_frame, text="Multiply", command=self.multiply_label)
        self.multiply_button.grid(row=1, column=6, padx=5, pady=5)

        # 変数の変更を監視
        self.initial_label_value.trace("w", self.on_option_change)
        self.low_offset_value.trace("w", self.on_option_change)
        self.high_offset_value.trace("w", self.on_option_change)

        # Matplotlibの図と軸を設定
        self.fig, self.ax = plt.subplots()
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        self.ax.set_aspect('equal', adjustable='datalim')
        self.fig.patch.set_facecolor('white')
        self.ax.set_facecolor('white')

        # Matplotlibの図をTkinterに埋め込む
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack(expand=True, fill='both')

        # イベントハンドラの設定
        self.cid_press = self.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_release = self.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.cid_scroll = self.canvas.mpl_connect('scroll_event', self.on_scroll)

        # --- データ & 描画オブジェクト ---
        self.x, self.y, self.labels = [], [], []
        self.z, self.x_q, self.y_q, self.z_q, self.w_q = [], [], [], [], []
        self.inner_map_x, self.inner_map_y = [], []
        self.outer_map_x, self.outer_map_y = [], []
        self.header = []
        
        # 描画オブジェクトを保持する変数
        self.points_collection = None
        self.line_collection = None
        self.label_texts = []
        self.map_plots = []

        self.last_path = None
        self.selected_point = None
        self.selected_line = None
        self.active_label = None
        self.edit_labels_start = None
        self.zoom_scale = 1.1
        self.pan_active = False
        self.pan_start = None
        self.selected_range_start = None
        self.selected_range_end = None
        self.selected_range_points = []
        self.dragging_range = False
        self.drag_start_pos = None

        self.undo_list = []
        self.redo_list = []

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.parent_dir = os.path.dirname(self.script_dir)
        self.save_undo_trajectory()

        self.default_map_path = self.parent_dir + '/csv/lane.csv'
        self.load_map(self.default_map_path)

    # (チェックボックスの排他制御など、変更のないメソッドは省略)
    def set_add_point(self):
        self.move_point_var.set(False)
        self.edit_label_var.set(False)
        self.delete_point_var.set(False)
        self.calculate_speed_var.set(False)
        self.move_selected_var.set(False)
        self.straight_line_var.set(False)

    def set_move_point(self):
        self.add_point_var.set(False)
        self.edit_label_var.set(False)
        self.delete_point_var.set(False)
        self.calculate_speed_var.set(False)
        self.move_selected_var.set(False)
        self.straight_line_var.set(False)

    def set_edit_label(self):
        self.add_point_var.set(False)
        self.move_point_var.set(False)
        self.delete_point_var.set(False)
        self.calculate_speed_var.set(False)
        self.move_selected_var.set(False)
        self.straight_line_var.set(False)

    def set_delete_point(self):
        self.add_point_var.set(False)
        self.move_point_var.set(False)
        self.edit_label_var.set(False)
        self.calculate_speed_var.set(False)
        self.move_selected_var.set(False)
        self.straight_line_var.set(False)

    def set_edit_labels(self):
        self.add_point_var.set(False)
        self.move_point_var.set(False)
        self.edit_label_var.set(False)
        self.delete_point_var.set(False)
        self.move_selected_var.set(False)
        self.straight_line_var.set(False)
    
    def set_move_selected(self):
        self.add_point_var.set(False)
        self.move_point_var.set(False)
        self.edit_label_var.set(False)
        self.delete_point_var.set(False)
        self.calculate_speed_var.set(False)
        self.straight_line_var.set(False)
        self.selected_range_start = None
        self.selected_range_end = None
        self.selected_range_points = []
        self.plot_data()
    
    def set_straight_line(self):
        self.add_point_var.set(False)
        self.move_point_var.set(False)
        self.edit_label_var.set(False)
        self.delete_point_var.set(False)
        self.calculate_speed_var.set(False)
        self.move_selected_var.set(False)
        self.selected_range_start = None
        self.selected_range_end = None
        self.selected_range_points = []
        self.plot_data()

    def add_label(self):
        if self.add_label_value.get() != 0:
            add_value = self.kmh_to_ms(float(self.add_label_value.get()))
            # numpy配列に変換して一括で加算
            self.labels = (np.array(self.labels) + add_value).tolist()
            self.plot_data()
            self.save_undo_trajectory()
    
    def sub_label(self):
        if self.add_label_value.get() != 0:
            sub_value = self.kmh_to_ms(float(self.add_label_value.get()))
            # numpy配列で効率的に計算
            labels_np = np.array(self.labels)
            labels_np[labels_np > sub_value] -= sub_value
            self.labels = labels_np.tolist()
            self.plot_data()
            self.save_undo_trajectory()
    
    def multiply_label(self):
        if self.multiply_label_value.get() > 0:
            multiply_value = float(self.multiply_label_value.get())
            self.labels = (np.array(self.labels) * multiply_value).tolist()
            self.plot_data()
            self.save_undo_trajectory()
    
    def on_option_change(self, *args):
        self.plot_data()

    def toggle_dark_mode(self):
        if self.dark_mode_var.get():
            face_color = 'black'
            tick_color = 'white'
        else:
            face_color = 'white'
            tick_color = 'black'
        
        self.fig.patch.set_facecolor(face_color)
        self.ax.set_facecolor(face_color)
        self.ax.tick_params(colors=tick_color)
        for label in self.ax.get_xticklabels() + self.ax.get_yticklabels():
            label.set_color(tick_color)

        self.plot_data()

    def load_csv(self, path=None):
        if path is None:
            file_path = filedialog.askopenfilename()
        else:
            file_path = path
        if not file_path: return

        # データをnumpy配列として読み込むと高速
        try:
            with open(file_path, 'r') as file:
                self.header = next(csv.reader(file))
            data = np.loadtxt(file_path, delimiter=',', skiprows=1)
            
            # 末尾の値と先頭の値が同じ場合は末尾の値を削除
            if np.array_equal(data[0,:2], data[-1,:2]):
                data = data[:-1]

            self.x = data[:, 0].tolist()
            self.y = data[:, 1].tolist()
            self.z = data[:, 2].tolist()
            self.x_q = data[:, 3].tolist()
            self.y_q = data[:, 4].tolist()
            self.z_q = data[:, 5].tolist()
            self.w_q = data[:, 6].tolist()
            self.labels = data[:, 7].tolist()
        except (IOError, ValueError) as e:
            print(f"Error loading CSV: {e}")
            return
        
        self.ax.set_xlim(min(self.x) - 5, max(self.x) + 5)
        self.ax.set_ylim(min(self.y) - 5, max(self.y) + 5)
        self.save_undo_trajectory()
        self.plot_data()
    
    def reset_view(self):
        if not self.x: return
        self.ax.set_xlim(min(self.x) - 5, max(self.x) + 5)
        self.ax.set_ylim(min(self.y) - 5, max(self.y) + 5)
        self.plot_data()

    def run(self):
        print("Running post-processing script...")
        path = self.script_dir + '/post/post.csv'
        self.save_csv(path)
        try:
            result = subprocess.run(['bash', self.script_dir + '/shell.sh', path], check=True, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("Subprocess completed successfully.")
                print(result.stdout)
        except subprocess.TimeoutExpired:
            print("Error: The subprocess timed out.")
        except subprocess.CalledProcessError as e:
            print(f"Error: {e.stderr}")

    def load_map(self, path=None):
        if path is None:
            file_path = filedialog.askopenfilename()
        else:
            file_path = path
        if not file_path: return
        
        self.inner_map_x, self.inner_map_y, self.outer_map_x, self.outer_map_y = [], [], [], []
        # ここもnumpyで読み込むと高速
        data = np.genfromtxt(file_path, delimiter=',', invalid_raise=False)
        self.inner_map_x = data[:, 0][~np.isnan(data[:, 0])].tolist()
        self.inner_map_y = data[:, 1][~np.isnan(data[:, 1])].tolist()
        self.outer_map_x = data[:, 2][~np.isnan(data[:, 2])].tolist()
        self.outer_map_y = data[:, 3][~np.isnan(data[:, 3])].tolist()

        # 古いマッププロットを削除
        for plot in self.map_plots:
            plot.remove()
        self.map_plots.clear()
        
        # 新しいプロットを追加
        plot1, = self.ax.plot(self.inner_map_x, self.inner_map_y, 'b-')
        plot2, = self.ax.plot(self.outer_map_x, self.outer_map_y, 'b-')
        self.map_plots.extend([plot1, plot2])
        
        self.plot_data()

    def save_csv(self, path=None):
        self.calc_quaternion()
        if path is None:
            file_path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
        else:
            file_path = path
        if not file_path: return
        
        self.last_path = file_path
        data_to_save = np.array([self.x, self.y, self.z, self.x_q, self.y_q, self.z_q, self.w_q, self.labels]).T
        
        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.header)
            writer.writerows(data_to_save)
            if self.x[0] != self.x[-1] or self.y[0] != self.y[-1]:
                writer.writerow(data_to_save[0])

    def plot_data(self):
        if not self.x:
            self.canvas.draw()
            return

        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        # 古い軌跡オブジェクトを削除
        if self.points_collection: self.points_collection.remove()
        if self.line_collection: self.line_collection.remove()
        for txt in self.label_texts: txt.remove()
        self.label_texts.clear()
        
        # 背景色とラベルの色を設定
        label_color = 'white' if self.dark_mode_var.get() else 'blue'

        # 点と線の色を計算
        point_colors = [self.get_color(label, i) for i, label in enumerate(self.labels)]
        line_colors = [self.get_color(self.labels[i]) for i in range(1, len(self.labels))]
        line_colors.append(self.get_color(self.labels[0]))

        # 点をscatterで一括描画
        self.points_collection = self.ax.scatter(self.x, self.y, c=point_colors, picker=5, zorder=3)

        # 線をLineCollectionで一括描画
        points = np.array([self.x, self.y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        # ループを閉じる線分を追加
        loop_segment = np.array([[self.x[-1], self.y[-1]], [self.x[0], self.y[0]]]).reshape(1, 2, 2)
        all_segments = np.concatenate([segments, loop_segment])
        
        self.line_collection = LineCollection(all_segments, colors=line_colors, zorder=2)
        self.ax.add_collection(self.line_collection)

        # ラベル表示
        if self.show_labels_var.get():
            for i in range(len(self.x)):
                txt = self.ax.text(self.x[i], self.y[i], f"{self.ms_to_kmh(self.labels[i]):.0f}",
                                   fontsize=12, ha='right', color=label_color, clip_on=True)
                self.label_texts.append(txt)

        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.canvas.draw_idle()

    def update_plot_for_motion(self):
        """マウス移動中の軽量なプロット更新"""
        if self.points_collection is None or self.line_collection is None:
            return

        # scatter（点）の座標を更新
        offsets = np.c_[self.x, self.y]
        self.points_collection.set_offsets(offsets)

        # LineCollection（線）のセグメントを更新
        points = offsets.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        loop_segment = np.array([[points[-1, 0, :], points[0, 0, :]]])
        all_segments = np.concatenate([segments, loop_segment])
        self.line_collection.set_segments(all_segments)

        # ラベルの位置を更新
        if self.show_labels_var.get():
            for i, txt in enumerate(self.label_texts):
                txt.set_position((self.x[i], self.y[i]))
        
        self.canvas.draw_idle()

    def get_color(self, label, index=None):
        if self.move_selected_var.get() and index is not None:
            if index == self.selected_range_start or index == self.selected_range_end:
                return 'orange'
        if self.straight_line_var.get() and index is not None:
             if index == self.selected_range_start or index == self.selected_range_end:
                return 'cyan'
        
        high_val = self.kmh_to_ms(self.high_offset_value.get())
        low_val = self.kmh_to_ms(self.low_offset_value.get())

        if label > high_val: return 'green'
        elif label > low_val: return 'yellow'
        else: return 'red'

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
                    self.active_label = point_idx
                    self.edit_label(point_idx)
            elif self.calculate_speed_var.get():
                # (元のロジックを維持)
                 if self.edit_labels_start is None:
                    self.edit_labels_start = self.find_nearest_point(event.xdata, event.ydata)
                 else:
                    end = self.find_nearest_point(event.xdata, event.ydata)
                    if end is not None:
                        new_label_kmh = simpledialog.askstring("Edit Label", f"Edit label for points {self.edit_labels_start} to {end}", initialvalue=f"{self.ms_to_kmh(self.labels[self.edit_labels_start]):.1f}")
                        if new_label_kmh is not None:
                            new_label_ms = self.kmh_to_ms(float(new_label_kmh))
                            indices = self.get_points_in_range(self.edit_labels_start, end)
                            for i in indices:
                                self.labels[i] = new_label_ms
                            self.plot_data()
                            self.edit_labels_start = None
                            self.save_undo_trajectory()

            elif self.move_selected_var.get():
                if not self.selected_range_start:
                    self.selected_range_start = self.find_nearest_point(event.xdata, event.ydata)
                    self.plot_data()
                elif not self.selected_range_end:
                    self.selected_range_end = self.find_nearest_point(event.xdata, event.ydata)
                    if self.selected_range_start is not None and self.selected_range_end is not None:
                        self.selected_range_points = self.get_points_in_range(self.selected_range_start, self.selected_range_end)
                        self.dragging_range = True
                        self.drag_start_pos = (event.xdata, event.ydata)
                        self.plot_data()
                else:
                    self.selected_range_start = self.find_nearest_point(event.xdata, event.ydata)
                    self.selected_range_end = None
                    self.selected_range_points = []
                    self.dragging_range = False
            elif self.straight_line_var.get():
                if self.selected_range_start is None:
                    self.selected_range_start = self.find_nearest_point(event.xdata, event.ydata)
                    self.plot_data()
                elif self.selected_range_end is None:
                    self.selected_range_end = self.find_nearest_point(event.xdata, event.ydata)
                    if self.selected_range_start is not None and self.selected_range_end is not None:
                        indices = self.get_points_in_range(self.selected_range_start, self.selected_range_end)
                        x0, y0 = self.x[self.selected_range_start], self.y[self.selected_range_start]
                        x1, y1 = self.x[self.selected_range_end], self.y[self.selected_range_end]
                        for i in indices:
                            # Skip start and end points
                            if i == self.selected_range_start or i == self.selected_range_end: continue
                            px, py = self.project_point_on_line(x0, y0, x1, y1, self.x[i], self.y[i])
                            self.x[i] = px
                            self.y[i] = py
                        self.plot_data()
                        self.save_undo_trajectory()
                        self.selected_range_start = None
                        self.selected_range_end = None
                else:
                    self.selected_range_start = self.find_nearest_point(event.xdata, event.ydata)
                    self.selected_range_end = None
                    self.plot_data()

        elif event.button == MouseButton.RIGHT:
            self.pan_active = True
            self.pan_start = (event.xdata, event.ydata)

    def on_release(self, event):
        # ドラッグ終了時に完全な再描画を実行
        if self.selected_point is not None or self.dragging_range:
            self.save_undo_trajectory()
            self.plot_data() # 色の更新などを反映

        self.selected_point = None
        self.selected_line = None
        self.pan_active = False
        self.dragging_range = False
        self.drag_start_pos = None

    def on_motion(self, event):
        if event.inaxes != self.ax: return

        if self.selected_point is not None:
            # 軽量な更新関数を呼び出す
            self.x[self.selected_point] = event.xdata
            self.y[self.selected_point] = event.ydata
            self.update_plot_for_motion()

        elif self.dragging_range and self.selected_range_points:
            # 範囲選択移動も軽量な更新
            if self.drag_start_pos is None: return
            dx = event.xdata - self.drag_start_pos[0]
            dy = event.ydata - self.drag_start_pos[1]
            
            x_np = np.array(self.x)
            y_np = np.array(self.y)
            
            # 選択範囲の点のインデックスをnumpy配列にする
            indices_np = np.array(self.selected_range_points)
            
            x_np[indices_np] += dx
            y_np[indices_np] += dy
            
            self.x = x_np.tolist()
            self.y = y_np.tolist()
            
            self.drag_start_pos = (event.xdata, event.ydata)
            self.update_plot_for_motion()

        elif self.pan_active:
            # パン機能
            if self.pan_start[0] is None or self.pan_start[1] is None: return
            dx = self.pan_start[0] - event.xdata
            dy = self.pan_start[1] - event.ydata
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            self.ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
            self.ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
            self.canvas.draw_idle()

    def on_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None: return
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        xdata, ydata = event.xdata, event.ydata

        scale_factor = self.zoom_scale if event.button == 'up' else 1 / self.zoom_scale
        
        new_xlim = [(x - xdata) * scale_factor + xdata for x in xlim]
        new_ylim = [(y - ydata) * scale_factor + ydata for y in ylim]

        self.ax.set_xlim(new_xlim)
        self.ax.set_ylim(new_ylim)
        self.canvas.draw_idle()

    def find_nearest_point(self, x, y):
        if not self.x: return None
        distances = np.hypot(np.array(self.x) - x, np.array(self.y) - y)
        min_idx = np.argmin(distances)
        # クリック許容範囲を軸の表示範囲に応じて動的に変更
        ax_range = np.hypot(self.ax.get_xlim()[1] - self.ax.get_xlim()[0], self.ax.get_ylim()[1] - self.ax.get_ylim()[0])
        if distances[min_idx] < ax_range * 0.01: # 画面範囲の1%
            return min_idx
        return None

    def find_nearest_line(self, x, y):
        if len(self.x) < 2: return None
        
        points = np.array([self.x, self.y]).T
        # ループを閉じる線を含めてセグメントを作成
        segments = np.array(list(zip(points, np.roll(points, -1, axis=0))))
        
        # クリックされた点
        click_point = np.array([x, y])
        
        # 各線分への垂線の足と距離を計算 (numpyで一括処理)
        line_starts, line_ends = segments[:, 0], segments[:, 1]
        line_vec = line_ends - line_starts
        line_len_sq = np.sum(line_vec**2, axis=1)
        
        # line_len_sq がゼロのセグメントを回避
        line_len_sq[line_len_sq == 0] = 1e-9

        vec_from_start = click_point - line_starts
        t = np.sum(vec_from_start * line_vec, axis=1) / line_len_sq
        t = np.clip(t, 0, 1)
        
        projected_points = line_starts + t[:, np.newaxis] * line_vec
        distances = np.hypot(*(projected_points - click_point).T)

        min_dist_idx = np.argmin(distances)
        ax_range = np.hypot(self.ax.get_xlim()[1] - self.ax.get_xlim()[0], self.ax.get_ylim()[1] - self.ax.get_ylim()[0])
        
        if distances[min_dist_idx] < ax_range * 0.02: # 許容範囲
            nearest_point = projected_points[min_dist_idx]
            insert_idx = (min_dist_idx + 1) % len(self.x)

            self.x.insert(insert_idx, nearest_point[0])
            self.y.insert(insert_idx, nearest_point[1])
            self.z.insert(insert_idx, 0.0) 
            self.labels.insert(insert_idx, self.kmh_to_ms(self.initial_label_value.get()))
            self.calc_quaternion() # クォータニオンも更新
            self.plot_data()
            self.save_undo_trajectory()
            return insert_idx
        return None

    def project_point_on_line(self, x0, y0, x1, y1, x, y):
        dx, dy = x1 - x0, y1 - y0
        if dx == 0 and dy == 0: return x0, y0
        t = ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)
        t = np.clip(t, 0, 1) # 線分上に制限
        px, py = x0 + t * dx, y0 + t * dy
        return px, py

    def delete_point(self, point_idx):
        self.x.pop(point_idx)
        self.y.pop(point_idx)
        self.z.pop(point_idx)
        self.labels.pop(point_idx)
        self.calc_quaternion()
        self.plot_data()
        self.save_undo_trajectory()

    def edit_label(self, point_idx):
        new_label = simpledialog.askstring("Edit Label", f"Edit label for point {point_idx}", initialvalue=f"{self.ms_to_kmh(self.labels[point_idx]):.1f}")
        if new_label is not None:
            try:
                self.labels[point_idx] = self.kmh_to_ms(float(new_label))
                self.plot_data()
                self.save_undo_trajectory()
            except ValueError:
                print("Invalid input for label.")
    
    def calc_quaternion(self):
        if len(self.x) < 2: return
        x_np, y_np = np.array(self.x), np.array(self.y)
        # 次の点の座標を取得 (最後の点は最初の点に接続)
        next_x = np.roll(x_np, -1)
        next_y = np.roll(y_np, -1)
        
        yaw = np.arctan2(next_y - y_np, next_x - x_np)
        
        # オイラー角からクォータニオンへ一括変換
        roll, pitch = 0.0, 0.0
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        self.w_q = (cy * cp * cr + sy * sp * sr).tolist()
        self.x_q = (cy * cp * sr - sy * sp * cr).tolist()
        self.y_q = (sy * cp * sr + cy * sp * cr).tolist()
        self.z_q = (sy * cp * cr - cy * sp * sr).tolist()


    def get_points_in_range(self, start_idx, end_idx):
        n = len(self.x)
        if start_idx == end_idx:
            return [start_idx]
        
        # 時計回りと反時計回りの距離を計算
        dist_forward = (end_idx - start_idx + n) % n
        dist_backward = (start_idx - end_idx + n) % n
        
        indices = []
        if dist_forward <= dist_backward:
            # 時計回り
            curr = start_idx
            while curr != end_idx:
                indices.append(curr)
                curr = (curr + 1) % n
            indices.append(end_idx)
        else:
            # 反時計回り
            curr = start_idx
            while curr != end_idx:
                indices.append(curr)
                curr = (curr - 1 + n) % n
            indices.append(end_idx)
            
        return indices
        
    def save_undo_trajectory(self):
        # 軌跡データをコピーして保存
        current_state = {
            'x': self.x.copy(), 'y': self.y.copy(), 'z': self.z.copy(),
            'labels': self.labels.copy(), 'x_q': self.x_q.copy(), 'y_q': self.y_q.copy(),
            'z_q': self.z_q.copy(), 'w_q': self.w_q.copy()
        }
        self.undo_list.append(current_state)
        if len(self.undo_list) > 50: # undo履歴の最大数を制限
            self.undo_list.pop(0)
        self.redo_list.clear()
    
    def undo(self):
        if len(self.undo_list) > 1:
            current_state = self.undo_list.pop()
            self.redo_list.append(current_state)
            
            previous_state = self.undo_list[-1]
            self.restore_state(previous_state)
            self.plot_data()
    
    def redo(self):
        if self.redo_list:
            next_state = self.redo_list.pop()
            self.undo_list.append(next_state)
            self.restore_state(next_state)
            self.plot_data()
    
    def restore_state(self, state):
        self.x = state['x'].copy()
        self.y = state['y'].copy()
        self.z = state['z'].copy()
        self.labels = state['labels'].copy()
        self.x_q = state['x_q'].copy()
        self.y_q = state['y_q'].copy()
        self.z_q = state['z_q'].copy()
        self.w_q = state['w_q'].copy()

    def ms_to_kmh(self, ms):
        return ms * 3.6
    
    def kmh_to_ms(self, kmh):
        return kmh / 3.6
    
    # (euler_from_quaternion, quaternion_from_euler は変更なしのため省略)
    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
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
    # 起動時にデフォルトのCSVを読み込む
    default_csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'post', 'post.csv')
    if os.path.exists(default_csv_path):
        plot_tool.load_csv(default_csv_path)
    root.mainloop()
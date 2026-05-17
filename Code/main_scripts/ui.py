import tkinter as tk
from tkinter import ttk, font
import cv2
import threading
from tkinter import filedialog
import time
import queue
import colour_monitor
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from PIL import Image, ImageTk

class UI:
    def __init__(self, root, cm1: colour_monitor.ColourMonitor, cm2: colour_monitor.ColourMonitor, abort_callback=None, kinetics_runner=None):
        self.root = root
        self.kinetics_runner = kinetics_runner

        self.root.title("Experiment Dashboard")
        
        self.cm1 = cm1
        self.cm2 = cm2
        self.abort_callback = abort_callback
        self.frame_delay = 30

        self.image_queue1 = queue.Queue(maxsize=2)
        self.image_queue2 = queue.Queue(maxsize=2)
        self._is_running = True
        self._updates_running = True
        
        # --- Style and Fonts ---
        self.style = ttk.Style(root)
        try:
            self.style.theme_use('winnative')
        except tk.TclError:
            self.style.theme_use('clam')

        try:
            font.nametofont("TkDefaultFont").configure(family="Segoe UI", size=10)
            self.title_font = font.Font(family="Segoe UI", size=14, weight="bold")
            self.status_font = font.Font(family="Segoe UI", size=10)
            self.log_font = font.Font(family="Consolas", size=10)
            self.button_font = font.Font(family="Segoe UI", size=11, weight="bold")
        except tk.TclError:
            font.nametofont("TkDefaultFont").configure(family="sans-serif", size=10)
            self.title_font = font.Font(family="sans-serif", size=14, weight="bold")
            self.status_font = font.Font(family="sans-serif", size=10)
            self.log_font = font.Font(family="Courier", size=10)
            self.button_font = font.Font(family="sans-serif", size=11, weight="bold")

        root.configure(bg="#f0f0f0")

        self.main_frame = ttk.Frame(root, padding="10 10 10 10")
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        self.left_frame = ttk.Frame(self.main_frame)
        self.right_frame = ttk.Frame(self.main_frame)
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Left side: camera feeds
        ttk.Label(self.left_frame, text="Filming Camera", font=self.title_font).pack(pady=(0, 5))
        self.camera_canvas1 = tk.Canvas(self.left_frame, bg="black", highlightthickness=0)
        self.camera_canvas1.pack(pady=(0, 10), fill=tk.BOTH, expand=True)
        
        ttk.Label(self.left_frame, text="Stirring Plate Camera", font=self.title_font).pack(pady=(10, 5))
        self.camera_canvas2 = tk.Canvas(self.left_frame, bg="black", highlightthickness=0)
        self.camera_canvas2.pack(pady=0, fill=tk.BOTH, expand=True)
        
        self.display_image1 = None
        self.display_image2 = None

        # Right side: graph and log
        self.fig = Figure(figsize=(5, 6), dpi=100, facecolor="#f0f0f0")
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)

        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor('#ffffff')
            ax.grid(True, linestyle='--', alpha=0.6, color='#cccccc')
            for spine in ax.spines.values():
                spine.set_edgecolor('#bbbbbb')
            ax.tick_params(colors='#333333', labelsize=9)
            ax.yaxis.label.set_color('#333333')
            ax.xaxis.label.set_color('#333333')
            ax.title.set_color('#333333')

        self.fig.tight_layout(pad=3.0)

        self.graph_canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.graph_canvas.get_tk_widget().pack(pady=(0, 10), fill=tk.BOTH, expand=True)

        self.bottom_right_frame = ttk.Frame(self.right_frame)
        self.bottom_right_frame.pack(fill=tk.BOTH, expand=True)

        self.info_pane = ttk.Frame(self.bottom_right_frame)
        self.info_pane.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.status_frame = ttk.Labelframe(self.info_pane, text="Monitor Status", padding="10")
        self.status_frame.pack(fill=tk.X, expand=False)

        self.monitor_status1 = ttk.Label(self.status_frame, text="Filming camera:\nWaiting for status...", justify=tk.LEFT, anchor="w", font=self.status_font)
        self.monitor_status1.pack(fill=tk.X, pady=5)
        self.monitor_status2 = ttk.Label(self.status_frame, text="Plate camera:\nWaiting for status...", justify=tk.LEFT, anchor="w", font=self.status_font)
        self.monitor_status2.pack(fill=tk.X, pady=(0,5))

        self.metrics_frame = ttk.Labelframe(self.info_pane, text="Kinetics Metrics", padding="10")
        self.metrics_frame.pack(fill=tk.X, expand=False, pady=(10, 0))

        try:
            italic_font = font.Font(family="Segoe UI", size=9, slant="italic")
        except tk.TclError:
            italic_font = font.Font(family="sans-serif", size=9, slant="italic")
        self.kinetics_source_label = ttk.Label(self.metrics_frame, text="Source: Filming Camera", justify=tk.LEFT, anchor="w", font=italic_font)
        self.kinetics_source_label.pack(fill=tk.X, pady=(0, 5))

        self.metrics_label = ttk.Label(self.metrics_frame, text="Waiting for data...", justify=tk.LEFT, anchor="w", font=self.status_font)
        self.metrics_label.pack(fill=tk.X, pady=(0, 5))

        self.log_frame = ttk.Frame(self.bottom_right_frame)
        self.log_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(self.log_frame, height=10, state=tk.DISABLED, font=self.log_font, bg="#ffffff", fg="#333333", relief=tk.SOLID, borderwidth=1, wrap=tk.WORD, padx=5, pady=5)
        self.log_scrollbar = ttk.Scrollbar(self.log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text['yscrollcommand'] = self.log_scrollbar.set
        self.log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 1. SET FULLSCREEN - Moved here to ensure it's applied after initial layout
        self.root.update_idletasks() # Ensure window is rendered before going fullscreen
        self.root.attributes('-fullscreen', True)
        self.root.bind("<Escape>", lambda e: self.root.attributes("-fullscreen", False))

        self.pending_status_updates = queue.Queue()
        self.root.after(50, self._process_pending_updates)

        # --- Control Buttons ---
        self.button_frame = ttk.Frame(self.right_frame)
        self.button_frame.pack(pady=(10, 0), fill=tk.X)

        self.style.configure('Abort.TButton', font=self.button_font, background='#c00000', foreground='white')
        self.style.map('Abort.TButton',
            background=[('active', '#e00000'), ('disabled', '#808080')],
            foreground=[('disabled', '#cccccc')])
        self.style.configure('TButton', font=self.button_font, padding=5)

        if self.abort_callback:
            self.abort_button = ttk.Button(self.button_frame, text="ABORT EXPERIMENT", command=self.abort_callback, style='Abort.TButton')
            self.abort_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 5))

        self.export_button = ttk.Button(self.button_frame, text="Export Graphs", command=self.export_graphs)
        self.export_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5, 0))

        self.frame_processor_thread = threading.Thread(target=self._frame_processor_loop, daemon=True)
        self.frame_processor_thread.start()
        self.update_camera_display()
        self.update_graph()

    def switch_kinetics_source(self):
        # callback for the 'Switch Graph Source' button
        if self.kinetics_runner:
            active_cam_name = self.kinetics_runner.switch_source()
            
            source_text = "Filming Camera" if active_cam_name == "cam1" else "Plate Camera"
            self.kinetics_source_label.config(text=f"Source: {source_text}")
            
            self.log_event(f"Kinetics graph source switched to {source_text}.")

    def programmatic_switch_kinetics_source(self):
        # thread-safe method to switch the kinetics graph source from the experiment logic
        if self.kinetics_runner:
            self.root.after_idle(self.switch_kinetics_source)

    def stop_updates(self):
        # stops the periodic updates of the graphs.
        self._updates_running = False

    def export_graphs(self):
        # opens a file dialog to save the current graphs as an image or CSV.
        filepath = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG Image", "*.png"), ("CSV Data", "*.csv"), ("JPEG Image", "*.jpg"), ("All files", "*.*")],
            title="Save Graphs As"
        )
    
        if not filepath:
            return  # User cancelled
        
        ext = filepath.lower().split('.')[-1]
        
        if '.' not in filepath:
            # default to CSV if user picked CSV in dialog
            filepath += ".csv"
            ext = "csv"

        if ext == "csv":
            self._export_data_as_csv(filepath)
        else:
            self._export_figure_as_image(filepath)
        
    def _export_figure_as_image(self, filepath):
        # saves the current figure as an image file.
        try:
            self.fig.savefig(filepath, dpi=300, bbox_inches='tight')
            self.log_event(f"Graphs exported to {filepath}")
        except Exception as e:
            self.log_event(f"Error exporting image: {e}")

    def _export_data_as_csv(self, filepath):
        # saves the graph data to a CSV file.
        import csv  # local import as it's only used here

        if not self.kinetics_runner:
            self.log_event("Cannot export CSV: Kinetics runner not available.")
            return

        try:
            data = self.kinetics_runner.get_graph_data()

            # extract data, providing empty lists as fallback
            conc_times = data.get('conc_times', [])
            conc_hues = data.get('conc_hues', [])
            conc_colours = data.get('conc_colours', [])
            rate_times = data.get('rate_times', [])
            rate_values = data.get('rate_values', [])

            num_rows = max(len(conc_times), len(rate_times))

            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                header = [
                    'concentration_time_s', 'normalized_hue', 'detected_colour',
                    'rate_time_s', 'rate_value_dH_dt'
                ]
                writer.writerow(header)

                for i in range(num_rows):
                    row = [
                        conc_times[i] if i < len(conc_times) else '',
                        conc_hues[i] if i < len(conc_hues) else '',
                        conc_colours[i] if i < len(conc_colours) else '',
                        rate_times[i] if i < len(rate_times) else '',
                        rate_values[i] if i < len(rate_values) else ''
                    ]
                    writer.writerow(row)

            self.log_event(f"Graph data exported to {filepath}")
        except Exception as e:
            self.log_event(f"Error exporting CSV: {e}")

    def destroy(self):
        # signal the background thread to stop
        self._is_running = False
        if hasattr(self, 'frame_processor_thread') and self.frame_processor_thread.is_alive():
            self.frame_processor_thread.join(timeout=1.0)

    def _frame_processor_loop(self):
        """
        Runs in a background thread. Fetches and processes frames for display.
        This prevents the UI thread from blocking on expensive image operations.
        """
        while self._is_running:
            frame1 = self.cm1.get_frame() if self.cm1 else None
            frame2 = self.cm2.get_frame() if self.cm2 else None

            if frame1 is not None:
                w, h = self.camera_canvas1.winfo_width(), self.camera_canvas1.winfo_height()
                if w > 1 and h > 1:
                    tk_img = self._process_frame_for_display(frame1, self.cm1, w, h)
                    try:
                        self.image_queue1.put_nowait(tk_img)
                    except queue.Full:
                        pass  # discard frame if UI is lagging

            if frame2 is not None:
                w, h = self.camera_canvas2.winfo_width(), self.camera_canvas2.winfo_height()
                if w > 1 and h > 1:
                    tk_img = self._process_frame_for_display(frame2, self.cm2, w, h)
                    try:
                        self.image_queue2.put_nowait(tk_img)
                    except queue.Full:
                        pass  # discard frame if UI is lagging

            time.sleep(1 / 30) # Aim for ~30 FPS processing

    def _process_frame_for_display(self, frame, monitor, w, h):
        # performs all CPU-heavy work to prepare a single frame for display
        frame_resized = cv2.resize(frame, (w, h))
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        
        if monitor:
            self._draw_overlay(frame_rgb, monitor.ui_state)
        
        img = Image.fromarray(frame_rgb)
        return ImageTk.PhotoImage(img)

    def update_camera_display(self):
        """
        Runs in the UI thread. Pulls pre-processed images from the queue and
        updates the canvas widgets. This is a very fast operation.
        """
        try:
            img1 = self.image_queue1.get_nowait()
            self.display_image1 = img1  # keep reference
            self.camera_canvas1.delete("all")
            self.camera_canvas1.create_image(0, 0, anchor=tk.NW, image=img1)
        except queue.Empty:
            pass  # no new frame is ready

        try:
            img2 = self.image_queue2.get_nowait()
            self.display_image2 = img2  # keep reference
            self.camera_canvas2.delete("all")
            self.camera_canvas2.create_image(0, 0, anchor=tk.NW, image=img2)
        except queue.Empty:
            pass  # no new frame is ready

        if self._is_running:
            self.root.after(self.frame_delay, self.update_camera_display)

    def update_graph(self):
        if self.kinetics_runner:
            data = self.kinetics_runner.get_graph_data()

            # --- update plots ---
            self.ax1.clear()
            self.ax2.clear()

            try:
                font_family = "Segoe UI" if "Segoe UI" in font.families() else "sans-serif"
            except tk.TclError:
                font_family = "sans-serif"

            self.ax1.set_title("Reaction Kinetics", fontdict={'family': font_family, 'size': 12})
            self.ax1.set_ylabel("Concentration (Norm. Hue)", fontdict={'family': font_family, 'size': 10})
            self.ax2.set_xlabel("Time (s)", fontdict={'family': font_family, 'size': 10})
            self.ax2.set_ylabel("Reaction Rate (|dH/dt|)", fontdict={'family': font_family, 'size': 10})
            for ax in [self.ax1, self.ax2]:
                ax.grid(True, linestyle='--', alpha=0.6, color='#cccccc')

            # concentration plot with segmented colors
            times = np.array(data['conc_times'])
            hues = np.array(data['conc_hues'])
            colors = data['conc_colours']
            color_map = {'green': 'g', 'red': 'r', 'yellow': 'y', 'none': '#cccccc'}
            if len(times) > 1:
                for i in range(len(times) - 1):
                    self.ax1.plot(times[i:i+2], hues[i:i+2], color=color_map.get(colors[i], '#cccccc'))

            # rate plot
            self.ax2.plot(data['rate_times'], data['rate_values'], color='orange')

            self.fig.tight_layout(pad=3.0)
            self.graph_canvas.draw()

            # --- update metrics ---
            metrics = data['metrics']
            induction = metrics.get('induction_s')
            mean_cycle = metrics.get('mean_cycle_s')
            metrics_text = (
                f"Elapsed: {metrics['elapsed_s']:.1f}s\n"
                f"Transitions: {metrics['transition_count']}\n"
                f"Oscillations: {metrics['oscillations']}\n"
                f"Induction: {f'{induction:.1f}s' if induction is not None else 'N/A'}\n"
                f"Mean Cycle: {f'{mean_cycle:.1f}s' if mean_cycle is not None else 'N/A'}")
            self.metrics_label.config(text=metrics_text)

        if self._updates_running:
            self.root.after(1000, self.update_graph)

    def _draw_overlay(self, frame, monitor_state):
        # --- always-on info ---
        current_color = monitor_state["voted_colour"].name
        confidence = monitor_state.get("confidence", 0.0)
        avg_rgb = monitor_state.get("avg_rgb", (0,0,0))

        if monitor_state["is_waiting"]:
            # --- calculations for waiting state ---
            timeout_pct = min(monitor_state["elapsed_s"] / monitor_state["timeout_s"], 1.0)
            streak_pct  = min(monitor_state["confirm_streak"] / monitor_state["confirmation_frames"], 1.0)
            waiting_for = monitor_state["waiting_for"].name if monitor_state["waiting_for"] else "N/A"
            
            # --- dynamic Y-positions and overlay height ---
            y_offset = 0
            overlay_height = 160 # Default height for the overlay rectangle
            
            # if voted_colour is UNKNOWN, we won't display confidence, so shift subsequent elements up.
            if monitor_state["voted_colour"] == colour_monitor.ColourState.UNKNOWN:
                y_offset = -20 # Shift up by 20 pixels (approx. one line height)
                overlay_height = 140 # Reduce overlay rectangle height

            # --- drawing (full overlay) ---
            overlay = frame.copy()
            alpha = 0.6
            cv2.rectangle(overlay, (10, 10), (350, overlay_height), (20, 20, 20), -1)
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

            # --- text ---
            cv2.putText(frame, f"Waiting for: {waiting_for}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
            cv2.putText(frame, f"Current: {current_color}", (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
            
            # only show confidence if a specific color is detected
            if monitor_state["voted_colour"] != colour_monitor.ColourState.UNKNOWN:
                cv2.putText(frame, f"Confidence: {confidence:.1%}", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)

            cv2.putText(frame, f"Avg RGB: {avg_rgb}", (20, 95 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # --- timeout progress bar ---
            cv2.putText(frame, "Timeout", (20, 120 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.rectangle(frame, (90, 110 + y_offset), (330, 125 + y_offset), (60, 60, 60), -1)
            bar_color = (0, 165, 255) if timeout_pct > 0.8 else (50, 205, 50)
            cv2.rectangle(frame, (90, 110 + y_offset), (90 + int(240 * timeout_pct), 125 + y_offset), bar_color, -1)

            # --- streak progress bar ---
            cv2.putText(frame, "Confirm", (20, 145 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.rectangle(frame, (90, 135 + y_offset), (330, 150 + y_offset), (60, 60, 60), -1)
            cv2.rectangle(frame, (90, 135 + y_offset), (90 + int(240 * streak_pct), 150 + y_offset), (135, 206, 250), -1)
        else:
            # --- drawing (smaller overlay for idle state) ---
            overlay = frame.copy()
            alpha = 0.6
            cv2.rectangle(overlay, (10, 10), (350, 60), (20, 20, 20), -1)
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

            # --- text ---
            cv2.putText(frame, "Status: Idle", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
            color_text = f"Color: {current_color}"
            if monitor_state["voted_colour"] != colour_monitor.ColourState.UNKNOWN:
                color_text += f" ({confidence:.0%})"
            cv2.putText(frame, color_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            rgb_text = f"RGB: {avg_rgb[0]},{avg_rgb[1]},{avg_rgb[2]}"
            cv2.putText(frame, rgb_text, (190, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    def queue_monitor_update(self, monitor_id, status_text):
        self.pending_status_updates.put((monitor_id, status_text))

    def _process_pending_updates(self):
        while not self.pending_status_updates.empty():
            monitor_id, status_text = self.pending_status_updates.get()
            if monitor_id == 1:
                self.monitor_status1.config(text=status_text)
            elif monitor_id == 2:
                self.monitor_status2.config(text=status_text)
        self.root.after(50, self._process_pending_updates)

    def log_event(self, event):
        try:
            # check if the widget still exists before trying to modify it.
            # this prevents a TclError if the UI has been destroyed during shutdown.
            if self.log_text.winfo_exists():
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, event + "\n")
                self.log_text.see(tk.END)
                self.log_text.config(state=tk.DISABLED)
            else:
                # fallback to console if UI is gone
                print(f"UI Log (widget destroyed): {event}")
        except tk.TclError:
            # this can happen if the window is destroyed between the check and the use.
            print(f"UI Log (TclError): {event}")
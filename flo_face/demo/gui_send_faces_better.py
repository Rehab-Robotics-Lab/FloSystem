import tkinter as tk
import math
import json
from serial_coms import SerialCom

# --- Helper functions from send_faces.py ---
def flatten(l):
    return [item for sublist in l for item in sublist]

def bytize(data):
    flat_data = flatten(data)
    data_bytes = [0]*math.ceil(len(flat_data)/8)
    for i in range(len(data_bytes)):
        for j in range(8):
            data_bytes[i] |= (flat_data[i*8 + j] << (7-j))
    return data_bytes
# ------------------------------------------

# Initialize serial communication
coms = SerialCom('/dev/ttyACM0', lambda *args: None)

class PixelGrid:
    def __init__(self, parent, cols, rows, x0, y0, cell=20):
        self.cols, self.rows = cols, rows
        self.cell = cell
        self.canvas = tk.Canvas(parent, width=cols*cell, height=rows*cell, bg='black')
        self.canvas.place(x=x0, y=y0)
        self.bits = [[0]*cols for _ in range(rows)]
        self._draw_grid()

    def _draw_grid(self):
        """Draws the entire grid based on self.bits."""
        self.canvas.delete("all")
        for r in range(self.rows):
            for c in range(self.cols):
                x, y = c*self.cell, r*self.cell
                color = 'white' if self.bits[r][c] else 'black'
                rect = self.canvas.create_rectangle(
                    x, y, x+self.cell, y+self.cell,
                    fill=color, outline='gray')
                self.canvas.tag_bind(rect, '<Button-1>', lambda ev, rr=r, cc=c: self.toggle(rr, cc))

    def toggle(self, r, c):
        self.bits[r][c] ^= 1
        self._draw_grid()

    def get_matrix(self):
        return self.bits

    def set_matrix(self, matrix):
        """Replace bits with matrix and redraw."""
        self.bits = [row[:] for row in matrix]
        self._draw_grid()

class FaceDesigner(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Face Designer")
        # Start maximized (windowed fullscreen)
        try:
            self.state('zoomed')
        except Exception:
            self.attributes('-zoomed', True)

        # Load predefined faces
        with open('faces_demo.json') as f:
            self.faces = json.load(f)

        # Manual draw grids
        self.right_eye = PixelGrid(self, 8, 8, x0=20, y0=20)
        self.left_eye  = PixelGrid(self, 8, 8, x0=480, y0=20)
        self.mouth     = PixelGrid(self, 16, 8, x0=90, y0=240, cell=30)

        # Built faces buttons
        face_frame = tk.LabelFrame(self, text="Built Faces")
        face_frame.place(x=1000, y=20)
        mouth_keys = list(self.faces.get('mouths', {}).keys())
        for idx, name in enumerate(mouth_keys):
            row = idx % 6
            col = idx // 6
            btn = tk.Button(face_frame, text=name, width=12,
                            command=lambda n=name: self.select_face(n))
            btn.grid(row=row, column=col, padx=5, pady=2)

        # Instructions
        instr = tk.Label(self, text="Click pixels to toggle. Press Enter to send manual face.")
        instr.place(x=20, y=600)
        self.bind('<Return>', self.send_manual_face)

    def select_face(self, name):
        """Load and send a predefined face by name."""
        face = self.faces['mouths'][name]
        mouth_matrix = face['on']
        eyes_name = face['eyes']
        eye_matrix = self.faces['eyes'][eyes_name]['center']['on']
        self.left_eye.set_matrix(eye_matrix)
        self.right_eye.set_matrix(eye_matrix)
        self.mouth.set_matrix(mouth_matrix)
        self.send_face_data(mouth_matrix, eye_matrix, eye_matrix)

    def send_manual_face(self, event=None):
        left = self.left_eye.get_matrix()
        right = self.right_eye.get_matrix()
        mouth = self.mouth.get_matrix()
        self.send_face_data(mouth, left, right)

    def send_face_data(self, mouth_matrix, left_matrix, right_matrix):
        mouth_bytes = [0] + bytize(mouth_matrix)
        left_bytes  = [1] + bytize(left_matrix)
        right_bytes = [2] + bytize(right_matrix)
        coms.sendData(mouth_bytes)
        coms.sendData(left_bytes)
        coms.sendData(right_bytes)
        print("Face sent ▶ mouth, left eye, right eye")

if __name__ == '__main__':
    app = FaceDesigner()
    app.mainloop()

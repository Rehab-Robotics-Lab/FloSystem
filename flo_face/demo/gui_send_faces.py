import tkinter as tk
import math
from serial_coms import SerialCom   # from send_faces.py :contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}

# --- Copy these helper functions from send_faces.py ---
def flatten(l): 
    return [item for sublist in l for item in sublist]

def bytize(data):
    flat_data = flatten(data)
    data_bytes = [0]*math.ceil(len(flat_data)/8)
    for i in range(len(data_bytes)):
        for j in range(8):
            data_bytes[i] |= (flat_data[i*8 + j] << (7-j))
    return data_bytes
# -----------------------------------------------------

# serial port (adjust to your device)
coms = SerialCom('/dev/ttyACM0', lambda *args: None)

class PixelGrid:
    def __init__(self, parent, cols, rows, x0, y0, cell=20):
        self.cols, self.rows = cols, rows
        self.cell = cell
        self.canvas = tk.Canvas(parent, width=cols*cell, height=rows*cell, bg='black')
        self.canvas.place(x=x0, y=y0)
        # create a 2D list of rect IDs and state bits
        self.bits = [[0]*cols for _ in range(rows)]
        for r in range(rows):
            for c in range(cols):
                x, y = c*cell, r*cell
                rect = self.canvas.create_rectangle(
                    x, y, x+cell, y+cell,
                    fill='black', outline='gray')
                # attach click handler
                self.canvas.tag_bind(rect, '<Button-1>', lambda ev, rr=r, cc=c: self.toggle(rr, cc))
        # note: bits[r][c] corresponds to on/off

    def toggle(self, r, c):
        self.bits[r][c] ^= 1
        color = 'white' if self.bits[r][c] else 'black'
        x, y = c*self.cell, r*self.cell
        # redraw that cell
        self.canvas.create_rectangle(
            x, y, x+self.cell, y+self.cell,
            fill=color, outline='gray')

    def get_matrix(self):
        return self.bits

# main application
class FaceDesigner(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Face Designer")
        self.geometry("600x300")
        # Eyes at top
        self.right_eye  = PixelGrid(self, 8, 8, x0=20,  y0=20)
        self.left_eye = PixelGrid(self, 8, 8, x0=480, y0=20)
        # Mouth below
        self.mouth     = PixelGrid(self, 16, 8, x0=90, y0=240, cell=30)
        # Instructions
        instr = tk.Label(self, text="Click pixels to toggle. Press Enter to send to robot.")
        instr.place(x=20, y=270)
        # bind Enter key
        self.bind('<Return>', self.send_face)

    def send_face(self, event=None):
        # build the three byte arrays exactly as send_faces.py does:
        meyes = self.left_eye.get_matrix()
        neyes = self.right_eye.get_matrix()
        mmouth = self.mouth.get_matrix()

        # serialize: [0]+mouth_bytes, [1]+left_eye_bytes, [2]+right_eye_bytes
        mouth_bytes = [0] + bytize(mmouth)
        left_bytes  = [1] + bytize(meyes)
        right_bytes = [2] + bytize(neyes)

        coms.sendData(mouth_bytes)
        coms.sendData(left_bytes)
        coms.sendData(right_bytes)
        print("Face sent ▶ mouth, left eye, right eye")

if __name__ == '__main__':
    app = FaceDesigner()
    app.mainloop()

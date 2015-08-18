# -*- coding: utf-8 -*-

from Tkinter import *
import tkFileDialog
from PIL import Image, ImageTk
from utility import *
import serial
import sys

COM_PORT = ['/dev/ttyUSB4',
            '/dev/ttyUSB5',
            '/dev/ttyUSB6',
            '/dev/ttyUSB7',
            'COM4',
            'COM5',
            'COM6',
            'COM7',
            'COM8']
BAUD_RATE = 9600
TIME_OUT = 0.001
GRID_COUNT = 32
SHUTTER_LIMIT = 22
sys.setrecursionlimit(3500)

class App:
    width = 800
    height = 800
    excess_width = 116
    title_text = "{}Floating Logo!{}".format(45 * ' ', 45 * ' ')
    def __init__(self, root):
        self.root = root
        self.root.title(self.title_text)
        self.root.geometry(self.getGeometry(self.width + self.excess_width, self.height))


        self.help_lbl = Label(self.root, text="Draw on the grids. Once it is a closed surface, press Print!", justify=CENTER, wraplength=80)
        self.help_lbl.grid(row=0, column=0)

        self.mode_btn = Button(self.root, text="Erase\n擦除", command=self.changeMode)
        self.mode_btn.grid(row=1, column=0)

        self.print_btn = Button(self.root, text="Print!\n打印", command=self.printFoam)
        self.print_btn.grid(row=2, column=0)

        self.reset_btn = Button(self.root, text="Draw again\n重画", command=self.reset)
        self.reset_btn.grid(row=3, column=0)

        self.open_shutter_btn = Button(self.root, text="Open shutters\n芝麻开门", command=self.openShutter)
        self.open_shutter_btn.grid(row=4, column=0)

        self.close_shutter_btn = Button(self.root, text="Close shutters\n芝麻关门", command=self.closeShutter)
        self.close_shutter_btn.grid(row=5, column=0)

        self.force_open_btn = Button(self.root, text="Force open shutters\n强制开门", command=self.forceOpen)
        self.force_open_btn.grid(row=6, column=0)

        self.force_close_btn = Button(self.root, text="Force close shutters\n强制关门", command=self.forceClose)
        self.force_close_btn.grid(row=7, column=0)

        self.force_stop_btn = Button(self.root, text="Force shutters\nto stop\n停止", command=self.forceStop)
        self.force_stop_btn.grid(row=8, column=0)

        self.hard_reset_btn = Button(self.root, text="Hard reset\nmachine\n重置", command=self.hardReset)
        self.hard_reset_btn.grid(row=9, column=0)

        self.check_ready_btn = Button(self.root, text="Check readiness\n检查Arduino", command=self.checkReady)
        self.check_ready_btn.grid(row=10, column=0)

        self.load_image_btn = Button(self.root, text="Load image\n打开图片", command=self.loadImage)
        self.load_image_btn.grid(row=11, column=0)

        self.canvas = Canvas(self.root,
                             width=self.width,
                             height=self.height)
        self.canvas.grid(row=0, column=1, rowspan=15)

        self.canvas.bind('<Motion>', self.motion)
        self.canvas.bind('<Button-1>', self.mouseDown)
        self.canvas.bind('<ButtonRelease-1>', self.mouseUp)

        self.createGrids(GRID_COUNT)

        self.isMouseDown = False        

        self.ser = None
        self.serial_msg = None
        self.initialiseSerial()

        self.toplevel = None

        self.eventLoop()

    def initialiseSerial(self):
        for port in COM_PORT:
            try:
                self.ser = serial.Serial(port, BAUD_RATE, timeout=TIME_OUT)
                print "Connected to", port
                break
            except:
                continue

        if self.ser is not None:
            self.probeArduino()
        else:
            print "serial com port not established."

    def printMsgFromArduino(self):
        if self.ser:
            serial_msg = self.ser.readline().strip()
            if serial_msg:
                self.serial_msg = serial_msg
                print self.serial_msg

    def eventLoop(self):
        if self.toplevel is None:
            self.rotateTitle()
        else:
            self.root.title("")
        self.printMsgFromArduino()
        self.root.after(100, self.eventLoop)

    def rotateTitle(self):
        rol = lambda l: l[1:] + l[:1]
        self.title_text = rol(self.title_text)
        self.root.title(self.title_text)

    def updateCanvas(self):
        for grid in self.grid_map.allGrids():
            if grid.isPrintable():
                self.canvas.itemconfig(grid.ID, fill = 'CYAN')
            else:
                self.canvas.itemconfig(grid.ID, fill = 'WHITE')

    def updateSingleGrid(self, coord):
        grid = self.grid_map.grid(*self.grid_map.pixelToGridCoord(coord))
        if grid.isOccupied():
            self.canvas.itemconfig(grid.ID, fill = 'CYAN')
        else:
            self.canvas.itemconfig(grid.ID, fill = 'WHITE')

    def createGrids(self, side_count):
        g_width = self.width / side_count
        g_height = self.height / side_count
        self.grid_map = GridMap(side_count, g_width)
        for i in range(side_count):
            for j in range(side_count):
                self.grid_map.grid(i, j).setID(self.canvas.create_rectangle(i * g_width,
                                                                            j * g_height,
                                                                            (i + 1) * g_width,
                                                                            (j + 1) * g_height,
                                                                            fill='WHITE', width=1))


    def getGeometry(self, w, h):
        ws = self.root.winfo_screenwidth()
        hs = self.root.winfo_screenheight()
        x = (ws/2) - (w/2)
        y = (hs/2) - (h/2)
        return '{}x{}+{}+{}'.format(w, h, x, y)

    def motion(self, event):
        if self.isMouseDown:
            self.grid_map.clicked((event.x, event.y))
            self.updateSingleGrid((event.x, event.y))
            #print 'x: {}, y: {}'.format(event.x, event.y)

    def mouseDown(self, event):
        self.isMouseDown = True
        self.grid_map.clicked((event.x, event.y))
        self.updateSingleGrid((event.x, event.y))

    def mouseUp(self, event):
        self.isMouseDown = False
        self.checkIfClosedSurface()

    def checkIfClosedSurface(self):
        self.surface = Surface(self.grid_map)
        self.surface.fillSurface()
        self.updateCanvas()
        #if self.surface.isClosedSurface():
            #self.surface.fillSurface()
            #self.updateCanvas()

    def reset(self):
        self.createGrids(GRID_COUNT)

    def openShutter(self):
        if self.ser is not None:
            if self.serial_msg == "ready":
                self.serial_msg = None
                self.serialWrite('O')
            else:
                print "Arduino is busy"
        else:
            print "Arduino is not connected"


    def closeShutter(self):
        if self.ser is not None:
            if self.serial_msg == "ready":
                self.serial_msg = None
                self.serialWrite('C')
            else:
                print "Arduino is busy"
        else:
            print "Arduino is not connected"

    def forceOpen(self):
        if self.ser is not None:
            if self.serial_msg == "ready":
                self.serial_msg = None
                self.serialWrite('F')
            else:
                print "Arduino is busy"
        else:
            print "Arduino is not connected"

    def forceClose(self):
        if self.ser is not None:
            if self.serial_msg == "ready":
                self.serial_msg = None
                self.serialWrite('X')
            else:
                print "Arduino is busy"
        else:
            print "Arduino is not connected"

    def forceStop(self):
        if self.ser is not None:
            if self.serial_msg != "ready":
                self.serial_msg = None
                self.serialWrite('S')
            else:
                print "Shutter is not moving!"
        else:
            print "Arduino is not connected"

    def hardReset(self):
        if self.ser is not None:
            if self.serial_msg == "ready":
                self.serial_msg = None
                self.serialWrite('H')
            else:
                print "Arduino is busy"
        else:
            print "Arduino is not connected"

    def checkReady(self):
        self.probeArduino()

    def loadImage(self):
        if self.toplevel is not None:
            self.toplevel.withdraw()

        try:
            image_file = tkFileDialog.askopenfile(initialdir="./sample pictures")
            self.image_handle = ImageTk.PhotoImage(Image.open(image_file.name).resize((self.width, self.height), Image.ANTIALIAS))

            if self.toplevel is not None:
                self.toplevel.destroy()

            self.setupOverlayingWindow()
        except:
            if image_file is not None:
                print "Invalid image"

        if self.toplevel is not None:
            self.toplevel.deiconify()
        
    def setupOverlayingWindow(self):
        self.toplevel = Toplevel(self.root,
                             width=self.width,
                             height=self.height)

        self.toplevel.title("Press the X >> to close the loaded image")

        self.toplevel.geometry('{}x{}+{}+{}'.format(self.width,
                                                    self.height,
                                                    self.root.winfo_x() + self.excess_width,
                                                    self.root.winfo_y()))
        
        self.toplevel.wait_visibility(self.toplevel)
        self.toplevel.wm_attributes("-alpha", 0.5)
        self.toplevel.wm_attributes("-topmost", 1)

        self.toplevel_canvas = Canvas(self.toplevel,
                                 width=self.width,
                                 height=self.height)

        self.toplevel_canvas.pack()

        self.toplevel_canvas.bind('<Motion>', self.motion)
        self.toplevel_canvas.bind('<Button-1>', self.mouseDown)
        self.toplevel_canvas.bind('<ButtonRelease-1>', self.mouseUp)

        self.toplevel_image = self.toplevel_canvas.create_image(0, 0, image=self.image_handle, anchor=N+W)

        self.toplevel.protocol("WM_DELETE_WINDOW", self.toplevelDestroyed)

    def toplevelDestroyed(self):
        self.toplevel.destroy()
        self.toplevel = None

    def changeMode(self):
        self.grid_map.erase_mode = not self.grid_map.erase_mode
        if self.grid_map.erase_mode:
            self.mode_btn.config(text="Draw\n画")
        else:
            self.mode_btn.config(text="Erase\n擦掉")

    def printFoam(self):
        self.surface.fillSurface()
        if not self.surface.isValidShape():
            print "Invalid shape"
        else:
            grids = self.surface.getGridForPrinting()

            if self.serial_msg == "ready":
                self.serial_msg = None
                self.serialWrite('B')
                for msg in self.getMsgForArduino(grids):
                    print msg
                    self.serialWrite('S{}E'.format(msg))
            else:
                if self.ser is None:
                    print "Arduino is not connected, but here's the output anyway:"
                    for msg in self.getMsgForArduino(grids):
                        print msg
                else:
                    print "Arduino is busy"

    def getMsgForArduino(self, grids):
        for row, lhs, rhs in self.getShutterSteps(grids):
            yield "-{:2}L{:2}R{:2}".format(row, lhs, rhs)

    def getShutterSteps(self, grids):
        for row in range(GRID_COUNT):
            for lhs in range(GRID_COUNT):
                if grids[row][lhs].isPrintable(): break
                    
            for rhs in range(GRID_COUNT):
                if grids[row][GRID_COUNT - 1 - rhs].isPrintable(): break

            # limit shutters to halfway point in empty row
            if lhs == GRID_COUNT - 1:
                lhs = rhs = GRID_COUNT / 2

            # limit shutters to limiting point in case the shape requires them to move further
            lhs = SHUTTER_LIMIT if lhs >= SHUTTER_LIMIT else lhs
            rhs = SHUTTER_LIMIT if rhs >= SHUTTER_LIMIT else rhs
            
            if lhs >= SHUTTER_LIMIT or rhs >= SHUTTER_LIMIT:
                print "Shutter steps restricted due to limited length."

            yield (row, lhs, rhs)

    def serialWrite(self, msg):
        if self.ser:
            self.ser.write(msg)

    def probeArduino(self):
        self.serialWrite('R')
        
root = Tk()
app = App(root)
root.mainloop()
try:
    if app.ser:
        app.ser.close()
    root.destroy()
except:
    pass

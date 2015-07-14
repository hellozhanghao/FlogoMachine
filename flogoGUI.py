from Tkinter import *
from utility import *
import serial

#COM_PORT = 'COM6'
COM_PORT = '/dev/ttyUSB4'
GRID_COUNT = 5

class App:
    width = 600
    height = 600
    title_text = "{}Floating Logo!{}".format(45 * ' ', 45 * ' ')
    def __init__(self, root):
        self.root = root
        self.root.title(self.title_text)
        #self.root.geometry(self.getGeometry(self.width + 200, self.height))


        self.print_btn = Button(self.root, text="Print!", command=self.printFoam)
        self.print_btn.grid(row=0, column=0)

        self.reset_btn = Button(self.root, text="Reset", command=self.reset)
        self.reset_btn.grid(row=1, column=0)

        self.help_lbl = Label(self.root, text="Draw on the grids. Once it is a closed surface, press Print!", justify=CENTER, wraplength=80)
        self.help_lbl.grid(row=2, column=0)

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
        self.initialiseSerial()

        self.eventLoop()
##        self.canvasLoop()

    def initialiseSerial(self):
        self.ser = serial.Serial(COM_PORT, 9600, timeout=0.01)

    def eventLoop(self):
        self.rotateTitle()
        self.root.after(100, self.eventLoop)
        asd = self.ser.read()
        if asd:
            print asd

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
        if grid.isPrintable():
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

    def mouseUp(self, event):
        self.isMouseDown = False
        self.checkIfClosedSurface()

    def checkIfClosedSurface(self):
        self.surface = Surface(self.grid_map)
        if self.surface.isClosedSurface():
            self.surface.fillSurface()
            self.updateCanvas()

    def reset(self):
        self.createGrids(GRID_COUNT)

    def printFoam(self):
        if not self.surface.isValidShape():
            print "Invalid shape"
        else:
            self.surface.fillSurface()
            grids = self.grid_map.grids
            if not self.surface.isHorizontallyConvexedSurface():
                # transpose matrix
                grids = zip(*grids)
                print 'transposed'

            self.ser.write('B')
            for msg in self.getMsgForArduino(grids):
                print msg
                self.ser.write('S{}E'.format(msg))

    def getMsgForArduino(self, grids, separator='\n'):
        step_list = []
        for row in range(GRID_COUNT):
            lhs = 0
            for lhs in range(GRID_COUNT):
                if grids[row][lhs].isPrintable(): break

            rhs = 0
            for rhs in range(GRID_COUNT - 1, -1, -1):
                if grids[row][rhs].isPrintable(): break

            rhs = GRID_COUNT - rhs - 1
            if lhs + rhs == GRID_COUNT * 2 - 2:
                lhs = GRID_COUNT / 2
                rhs = GRID_COUNT / 2 + 1
            elif lhs + rhs > GRID_COUNT:
                difference = lhs + rhs - GRID_COUNT
                if lhs > rhs:
                    lhs -= difference
                else:
                    rhs -= difference
            yield "-{:2}L{:2}R{:2}".format(row, lhs, rhs)
        
root = Tk()
app = App(root)
root.mainloop()
try:
    if app.ser:
        app.ser.close()
    root.destroy()
except:
    pass

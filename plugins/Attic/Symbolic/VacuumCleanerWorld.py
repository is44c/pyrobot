import Tkinter, os, pickle
import Image, ImageTk, ImageDraw, ImageFont 

class GUI(Tkinter.Toplevel):
    """
    A simple world from Russell and Norvig's AIMA. This works
    in tandom with SymbolicServer.
    """
    def __init__(self, root, width, height):
        Tkinter.Toplevel.__init__(self, root)
        self.done = 0
        self.quit = 0
        self.root = root
        self.width = width
        self.height = height
        self.title("SymbolicSimulator: VacuumCleanerWorld")
        self.canvas = Tkinter.Canvas(self,width=self.width,height=self.height,bg="white")
        self.canvas.pack()
        self.winfo_toplevel().protocol('WM_DELETE_WINDOW',self.destroy)
        self.world = {"A": "Dirty", "B": "Clean"}
        self.location = "A"
        self.dirtFilename = os.environ["PYRO"] + "/images/dirt.gif" 
        self.vacFilename = os.environ["PYRO"] + "/images/vac.gif" 
        self.dirtImage = Image.open(self.dirtFilename)
        self.vacImage = Image.open(self.vacFilename)
        self.vacImageTk = ImageTk.PhotoImage(self.vacImage)
        self.dirtImageTk = ImageTk.PhotoImage(self.dirtImage)
        self.notSetables = ["location", "status"]
        self.updateables = ["location", "status"]
        self.movements = ["left", "right", "suck", "dump"]
        self.redraw()
    
    def process(self, request):
        retval = "error"
        if request == 'right':
            if self.location == 'A':
                self.location = 'B'
            retval = "ok"
        elif request == 'left':
            if self.location == 'B':
                self.location = 'A'
            retval = "ok"
        elif request == 'suck':
            self.world[self.location] = "Clean"
            retval = "ok"
        elif request == 'dump':
            self.world[self.location] = "Dirty"
            retval = "ok"
        elif request == 'location':
            retval = self.location
        elif request == 'status':
            retval = self.world[self.location]
        elif request == 'end' or request == 'exit':
            retval = "ok"
            self.done = 1
        elif request == 'quit':
            retval = "ok"
            self.done = 1
            self.quit = 1
        elif request == 'notsetables':
            retval = self.notSetables
        elif request == 'updateables':
            retval = self.updateables
        elif request == 'movements':
            retval = self.movements
        else:   # unknown command; returns "error"
            pass
        self.redraw()
        return pickle.dumps(retval)

    def redraw(self):
        self.canvas.delete('all')
        if self.location == 'A':
            self.canvas.create_image(0, 0, image = self.vacImageTk, anchor=Tkinter.NW)
        else:
            self.canvas.create_image(200, 0, image = self.vacImageTk, anchor=Tkinter.NW)
        if self.world["A"] == "Dirty":
            self.canvas.create_image(0, 100, image = self.dirtImageTk, anchor=Tkinter.NW)
        if self.world["B"] == "Dirty":
            self.canvas.create_image(200, 100, image = self.dirtImageTk, anchor=Tkinter.NW)

        self.canvas.create_line(200, 0, 200, 200, width = 2, fill = "black")

    def destroy(self):
        self.done = 1 # stop processing requests, if handing
        self.quit = 1 # stop accept/bind toplevel
        self.root.quit() # kill the gui

def INIT():
    root = Tkinter.Tk()
    root.withdraw()
    return GUI(root, 400, 200)
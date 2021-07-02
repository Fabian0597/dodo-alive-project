import sys
import pathlib

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../rbdl-orb/build/python/')

import rbdl
import numpy as np
import os

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox, CheckButtons
import time

plt.ion()


class PlayBackableObject:
    def __init__(self, f, interpolator=None):
        """provides functions to store and retrieve information for plot playback

        Args:
            f (function([Any] : last element, Double : Time)): function called when getAtTime is called
            interpolator (function[element, element], optional): defines how to interpolate between two given values. None -> no interpolation
        """
        self.__store = []
        self.__f = f
        self.__interpolatorF = interpolator

    def store(self, t, el):
        """store el at time t

        Args:
            t (Double): time
            el (Any): element to store
        """
        # TODO: check if sort required
        self.__store.append([t, el])

    def getAtTime(self, t):
        """retrieve last element bevore given time. retrieves multiple elements if stored at the same time

        Args:
            t (Double): Time
        """
        # TODO: sort required?
        lastEl = []
        saveT = 0
        for el in self.__store:
            if el[0] < t:
                # two stores at same position?
                if saveT == el[0]:
                    lastEl.append(el[1])
                else:
                    saveT = el[0]
                    lastEl = [el[1]]
            else:
                # found. interpolate
                if self.__interpolatorF is not None:
                    if el[0] != saveT:
                        fraction = 1 - (t - saveT) / (el[0] - saveT)
                    else:
                        fraction = 1
                    lastEl = self.__interpolatorF(fraction, lastEl, [el[1]])
                    break
        # none found
        self.__f(lastEl, saveT)


class PointPlotter(PlayBackableObject):
    def __init__(self, ax, style='rx', interpolate=False):
        """Used for plotting points.

        Args:
            ax (matplotlib.pyplot.axis): axis for plot
            style (str, optional): matplotlib style for lines/points: (eg. point: 'rx' are red poitns marked as x. 'ko-' is a black line with dots as point indicator). Defaults to 'rx'.
            interpolate (Bool, optional): defines if playback should interpolate between save values. Default is False
        """
        PlayBackableObject.__init__(self, self.__plotAtTime, self.__interpolator if interpolate else None)

        self.plot = None
        self.style = style
        self.ax = ax

    def addPoints(self, t, args):
        """view and store points

        Args:
            t (Double): time
            args (np.array(3), np.array(3), ...): points to visualize. currently 3d-array but only 2 dimensions are used (but could also be a 2d array)
        """
        if self.plot == None:
            self.plot, = self.ax.plot(0, 0, self.style)

        points = np.zeros((3, len(args)))
        for i, point in enumerate(args):
            points[0:len(point), i] = point

        self.store(t, points)
        self.plot.set_xdata(points[0])
        self.plot.set_ydata(points[1])  # TODO:3d?

    def __interpolator(self, fraction, oldVal, newVal):
        if len(oldVal) == 0 or len(newVal) == 0:
            return oldVal

        ret = []
        for i in range(0, len(oldVal)):
            if i < len(newVal):
                ret.append((oldVal[i] - newVal[i]) * fraction + newVal[i])
            else:
                ret.append(oldVal[i])

        return ret

    def __plotAtTime(self, lastEls, t):
        if len(lastEls) == 0:  # empty
            self.plot.set_xdata([])
            self.plot.set_ydata([])  # TODO:3d?
            return

        for el in lastEls:
            self.plot.set_xdata(el[0])
            self.plot.set_ydata(el[1])  # TODO:3d?


class ConsolePrinter(PlayBackableObject):
    replaceString = "; "
    lastPrint = ""

    def __init__(self):
        """print to console and store
        """
        PlayBackableObject.__init__(self, self.__printAtTime)

    def add(self, t, text):
        """add line

        Args:
            t (Double): time
            text (String): print and store text as string. newlines are converted to self.replaceString which is "; " per default
        """
        self.store(t, str(text))

    def __printAtTime(self, lastEls, t):
        if len(lastEls) == 0:
            return
        lastEl = ";".join(lastEls)
        printText = "at t: " + str(t) + " -> " + lastEl.replace("\n", self.replaceString)
        if printText != self.lastPrint:
            print(printText)
        self.lastPrint = printText


class VectorPlotter(PlayBackableObject):
    def __init__(self, ax, color='r'):
        """used for storing and plotting vectors

        Args:
            ax (matplotlib.pyplot.axis): axis for plot
            color (str, optional): Color for Vector. Defaults to 'r'.
        """
        PlayBackableObject.__init__(self, self.__plotAtTime)
        self.ax = ax
        self.plot = None
        self.color = color

    def addVector(self, t, origin, vector):
        """store and plot vector

        Args:
            t (Double): time
            origin (np.array(2)): [x,y] of vector origin
            vector (np.array(2)): [u,v] of vector
        """
        if self.plot == None:
            self.plot = self.ax.quiver([0, 0], [0, 0], [1], [1], color=self.color, minlength=0.001)

        self.plot.set_offsets(origin)
        self.plot.set_UVC([vector[0]], [vector[1]])
        self.store(t, [origin, vector])

    def __plotAtTime(self, lastEls, t):
        if len(lastEls) == 0:
            self.plot.set_UVC([0], [0])
            return
        for el in lastEls:
            self.plot.set_offsets(el[0])
            self.plot.set_UVC([el[1][0]], [el[1][1]])


class RobotPlotter:
    def __init__(self, model, stopFunction=None):
        """live and playback plotter for robots, points, vectors and prints. (matlabversion of meshup)
            for plotting robots:
                use addBodyId() and updateRobot()
            for plotting points:
                use showPoints()
            for plotting vectos:
                use showVector()
            for plotting prints:
                use print()

        Args:
            model (rbdl model): model
            stopFunction (function): if set, add a button to the matplotlib view which is called when clicked (typically stopps solver)
        """
        self.__model = model

        # init matplotlib
        self.__fig = plt.figure()
        self.__fig.canvas.set_window_title("RobotPlotter")
        self.__ax = self.__fig.add_subplot(111)
        self.setAxisLimit((-2, 40), (-2, 10))

        # init plotter/printers
        self.__robotPlotter = PointPlotter(self.__ax, style='ko-', interpolate=True)
        self.__pointPlotters = dict()
        self.__consolePrinter = ConsolePrinter()
        self.__vectorPrinters = dict()

        self.__tmax = 0

        self.__bodyIds = []

        # STOP BUTTON (if stop-function is defined)
        self.__stop_button_ax = None
        if stopFunction is not None:
            self.__stop_button_ax = self.__fig.add_axes([0.02, 0.93, 0.08, 0.04])
            self.__stop_button = Button(self.__stop_button_ax, 'STOP', hovercolor='0.975')

            def stop_button_on_clicked(mouse_event):
                stopFunction()

            self.__stop_button.on_clicked(stop_button_on_clicked)

    def setAxisLimit(self, x, y):
        """sets the limits of matplotlib axis

        Args:
            x (Double): xlim
            y (Double): ylim
        """
        self.__ax.set(xlim=x, ylim=y)  # todo: limits change?

    def addBodyId(self, id):
        """add rbdl body for visualization. is required since there is no easy way to get all bodies from rbdl

        Args:
            id (Int): bodyId (e.g model.GetBodyId(bodyName))
        """
        self.__bodyIds.append(id)

    def updateRobot(self, t, q):
        """update robot state

        Args:
            t (Double): time
            q (np.array(model.q_size)): q values
        """
        points = []
        for id in self.__bodyIds:
            points.append(rbdl.CalcBodyToBaseCoordinates(self.__model, q, id, np.zeros(3), True))
        self.__robotPlotter.addPoints(t, points)
        self.__updatePlot()

        if t > self.__tmax:
            self.__ax.set_title("t: " + str(t))
            self.__tmax = t

    def showPoints(self, t, style="kx", *args):
        """show and store points for playback. if new points with same style as previosly used are stored, old points are hidded. multiple points are given as aditional args

        Args:
            t (Double): time
            style (String): matplotlib style for lines/points: (eg. point: 'rx' are red poitns marked as x. 'ko-' is a black line with dots as point indicator).
            args (np.array(2)): x and y pint
        """
        if style not in self.__pointPlotters:
            self.__pointPlotters[style] = PointPlotter(self.__ax, style)

        self.__pointPlotters[style].addPoints(t, args)
        self.__updatePlot()

        if t > self.__tmax:
            self.__ax.set_title("t: " + str(t))
            self.__tmax = t

    def showVector(self, t, origin, vector, color='r'):
        """show and store vector. If color has been previously used for other vector, old vector is hidden

        Args:
            t (Double): time
            origin (np.array(2)): x and y value of vector origin
            vector (np.array(2)): u and v value of vector
            color (String, optional): Color for vector e.g. 'r','g','b','k',... Defaults to 'r'.
        """
        if color not in self.__vectorPrinters:
            self.__vectorPrinters[color] = VectorPlotter(self.__ax, color)
        self.__vectorPrinters[color].addVector(t, origin, vector)
        self.__updatePlot()

    def print(self, t, text):
        """print and store string

        Args:
            t (Double): time
            text (String): text to be printed. for playback newlines are replaced witch character defined in printer (default value is "; ")
        """
        self.__consolePrinter.add(t, text)
        print(text)

    def playbackMode(self):
        """initialize playbackmode UI
        """
        self.__initUI()
        plt.show(block=True)

    def __initUI(self):
        self.__ax.set_title("Playback-Mode")

        # remove STOP button
        if self.__stop_button_ax is not None:
            self.__stop_button_ax.remove()

        # OPTIONS CHECKBOXES
        self.__options_axes = self.__fig.add_axes([0.01, 0.3, 0.1, 0.15])
        self.__options = {
            "Robot": True,
            "Points": True,
            "Print": True,
            "Forces": True
        }
        self.__options_field = CheckButtons(self.__options_axes, list(self.__options.keys()),
                                            list(self.__options.values()))

        def options_clicked(label):
            # clear plot
            saveVal = self.__time_slider.val
            self.__time_slider.set_val(-1000000)
            # update
            self.__options[label] = not self.__options[label]
            self.__time_slider.set_val(saveVal)

        self.__options_field.on_clicked(options_clicked)

        # TIME SLIDER
        def sliders_on_changed(t):
            plotters = []
            if self.__options["Robot"] is True:
                plotters.append(self.__robotPlotter)
            if self.__options["Points"] is True:
                plotters += list(self.__pointPlotters.values())
            if self.__options["Print"] is True:
                plotters.append(self.__consolePrinter)
            if self.__options["Forces"] is True:
                plotters += list(self.__vectorPrinters.values())
            for plotter in plotters:
                plotter.getAtTime(t)
            self.__updatePlot()

        self.__time_slider_ax = self.__fig.add_axes([0.17, 0.02, 0.65, 0.03])
        self.__time_slider = Slider(self.__time_slider_ax, 'Time', 0, self.__tmax, valinit=self.__tmax)
        self.__time_slider.on_changed(sliders_on_changed)

        # START BUTTON
        start_button_ax = self.__fig.add_axes([0.9, 0.018, 0.08, 0.04])
        self.__start_button = Button(start_button_ax, 'Play', hovercolor='0.975')

        def start_button_on_clicked(mouse_event):
            if self.__time_slider.val >= self.__time_slider.valmax:
                self.__time_slider.set_val(self.__time_slider.valmin)
            startVal = self.__time_slider.val
            factor = self.__playSpeed
            startSysTime = time.time()
            t = self.__time_slider.valmin
            while t < self.__time_slider.valmax:
                # todo wrong for changing
                if factor != self.__playSpeed:
                    # startSysTime -=
                    pass
                t = (startVal + (time.time() - startSysTime)) * self.__playSpeed

                # update axis
                self.__time_slider.set_val(t)

        self.__start_button.on_clicked(start_button_on_clicked)

        # SPEED TEXTBOX
        self.__playSpeed = 1

        def speed_sliders_on_changed(speed):
            self.__playSpeed = speed

        speed_time_slider_ax = self.__fig.add_axes([0.93, 0.2, 0.025, 0.65])
        self.__speed_time_slider = Slider(speed_time_slider_ax, 'Speed', 0, 2.0, valinit=self.__playSpeed,
                                          orientation='vertical')
        self.__speed_time_slider.on_changed(speed_sliders_on_changed)

        # MESHUP BUTTON
        meshup_button_ax = self.__fig.add_axes([0.02, 0.018, 0.08, 0.04])
        self.__meshup_button = Button(meshup_button_ax, 'Meshup', hovercolor='0.975')

        def meshup_button_on_clicked(mouse_event):
            os.system(
                "meshup " + basefolder + "/articulated_leg.lua " + basefolder + "/animation.csv " + basefolder + "/forces.ff")

        self.__meshup_button.on_clicked(meshup_button_on_clicked)

    def __updatePlot(self):
        if 1 == 0:  # autoaxis
            self.__ax.relim()
            self.__ax.autoscale_view()
        self.__fig.canvas.draw()
        self.__fig.canvas.flush_events()


# example
if __name__ == "__main__":
    import time

    print("test called")
    # Create a new model
    model = rbdl.Model()
    joint_rot_y = rbdl.Joint.fromJointType("JointTypeRevoluteY")
    body = rbdl.Body.fromMassComInertia(
        1.,
        np.array([0., 0.5, 0.]),
        np.eye(3) * 0.05)
    xtrans = rbdl.SpatialTransform()
    xtrans.r = np.array([0., 1., 0.])
    body_1 = model.AppendBody(rbdl.SpatialTransform(), joint_rot_y, body)
    body_2 = model.AppendBody(xtrans, joint_rot_y, body)
    body_3 = model.AppendBody(xtrans, joint_rot_y, body)

    # init RobotPlotter
    plotter = RobotPlotter(model)
    for id in [body_1, body_2, body_3]:
        plotter.addBodyId(id)

    plotter.showPoints(0, 'rx', [0, 1], [2, 3])
    plotter.updateRobot(0, np.array([0, 0.2], dtype=np.double))
    plotter.showVector(0, [0, 0], [1, 1], 'b')
    time.sleep(1)
    plotter.showPoints(1, 'bo-', [0.5, 1], [2.5, 3])
    plotter.updateRobot(1, np.array([0, 0.3], dtype=np.double))
    plotter.showVector(1, [0, 0], [-2, -1], 'b')
    plotter.print(1, "Punkt hinzugefügt")
    plotter.print(1, "Zweiter Print")
    time.sleep(1)
    plotter.showPoints(2, 'bo-', [2.5, 1], [4, 3])
    plotter.showVector(2, [1, 0], [2, 2], 'r')
    plotter.updateRobot(2, np.array([0, 0.4], dtype=np.double))
    plotter.updateRobot(5, np.array([0, 0.4], dtype=np.double))

    plotter.playbackMode()

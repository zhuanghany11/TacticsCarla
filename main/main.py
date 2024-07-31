print("hello")

import sys
sys.path.append('/TacticsCarla/tactics2d')
print(sys.path)

import carla
print("libcarla: ", carla)

import tactics2d
print("tactics2d version: ", tactics2d.__version__)



import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from tactics2d.map.parser import OSMParser, XODRParser
from tactics2d.traffic import ScenarioDisplay



def zoom_factory(ax,base_scale = 2.):
    def zoom_fun(event):
        # get the current x and y limits
        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        cur_xrange = (cur_xlim[1] - cur_xlim[0])*.5
        cur_yrange = (cur_ylim[1] - cur_ylim[0])*.5
        xdata = event.xdata # get event x location
        ydata = event.ydata # get event y location
        if event.button == 'up':
            # deal with zoom in
            scale_factor = 1/base_scale
        elif event.button == 'down':
            # deal with zoom out
            scale_factor = base_scale
        else:
            # deal with something that should never happen
            scale_factor = 1
            print(event.button)
        # set new limits
        ax.set_xlim([xdata - cur_xrange*scale_factor,
                     xdata + cur_xrange*scale_factor])
        ax.set_ylim([ydata - cur_yrange*scale_factor,
                     ydata + cur_yrange*scale_factor])
        plt.draw() # force re-draw
        
    fig = ax.get_figure() # get the figure of interest
    # attach the call back
    fig.canvas.mpl_connect('scroll_event',zoom_fun)

    #return the function
    return zoom_fun


def test_xodr_parser(map_path, img_path):
    map_root = ET.parse(map_path).getroot()
    map_parser = XODRParser()
    map_ = map_parser.parse(map_root)

    fig, ax = plt.subplots()
    fig.set_layout_engine("constrained")
    ax.set_aspect("equal")
    # ax.set_axis_off()
    f = zoom_factory(ax, 2.0)

    scenario_display = ScenarioDisplay()
    scenario_display.display_map(map_, ax)
    ax.plot()
    # fig.savefig(img_path, facecolor="black")
    # fig.show()
    plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
    plt.show()
    # fig.savefig(img_path, facecolor="black")

if __name__ == "__main__":
    print("hello")
    test_xodr_parser("./carla_town/Town01.xodr",  "map.png")
    # test_xodr_parser("../tactics2d/tests/cases/XodrSamples/cross.xodr",  "map.png")


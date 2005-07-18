# Usage:
# * For pyrobot developers (needs swig):
#       python setup.py build
# * For pyrobot users (does not need swig):
#       python setup.py build_noswig
#       python setup.py install (needs write permissions to /usr/lib/python)
#
#    How to add a new camera device under directory <new_device>:
#    - add the <new_device> directory in the packages to be added in setup()
#    - add the <new_device> sources (*.c,*.cpp) in the pyrobot.camera.device._devices
#      Extension in setup() below
#    - add 2 lines in camera/device/Device.i:
#      1. #include "new_device.h"
#      2. %import "new_device.h"
#
#   Todo:
#       - add extensions for:
#           - pyrobot.camera.bt848
#           - pyrobot.camera.V4L
#           - pyrobot.robot.driver.saphira
#           - pyrobot.simulators.khepera.*
#           - pyrobot.tools.cluster
#       - check which of the macros/libs/libdirs are needed
#         (currently none under my Cygwin installation)
#       ? automate adding devices


import os, glob
from distutils.core import setup
from extradistutils import AutoIncludeExtension, \
                           BuildNoSwig, \
                           BuildExtension, \
                           BuildExtensionNoSwig

PYROBOT_VERSION = "4.0.0"

try:
    # write version file
    f = open("system/version.py", "w")
    print >> f, "# This file is automatically generated - do not edit"
    print >> f, "def version(): return %r" % PYROBOT_VERSION
    f.close()
except:
    print "Can't make system/version.py file!"

#std_macros = [("_POSIX_THREADS", None),
#              ("_POSIX_THREAD_SAFE_FUNCTIONS", None),
#              ("_REENTRANT", None),
#              ("POSIX", None),
#              ("__x86__", None),
#              ("__linux__", None),
#              ("__OSVERSION__", 2),
#              ("USINGTHREADS", None),
#              ("LINUX", None),
#              ("_GNU_SOURCE", None)]
#
include_dirs = ["/usr/local/include"]
library_dirs = ["/usr/local/lib"] #/usr/X11R6/lib"]

#std_libraries = ["stdc++", "X11", "Xt", "Xm", "dl", "pthread", "Xp", "Xext"]


setup(
    # metadata
    name = "pythonrobotics",
    version = PYROBOT_VERSION,
    description = "Robotics packages including vision, neural net, and "
                  "genetic algorithm components in Python and C",
    url = "http://emergent.brynmawr.edu/~dblank/pyrobot/",
    author = "Doug Blank, et al",
    author_email = "dblank@cs.brynmawr.edu",
    package_dir = {"" : ".."},
    scripts = filter(os.path.isfile, glob.glob(os.path.join("bin","*"))),

    # Python packages
    packages = ["pyrobot"] + [
        "pyrobot.%s" % package for package in
        # brain
        "brain", "brain.behaviors", "brain.psom", "brain.VisConx",
        # camera
        "camera", "camera.device", "camera.fake", "camera.blob",
        "camera.aibo", "camera.robocup",
        # vision
        "vision", "vision.cvision", "vision.example",
        # robot
        "robot", "robot.driver", "robot.driver.khepera", "robot.driver.b21r",
        # simulators
        "simulators", "simulators.khepera", "simulators.khepera.CNTRL",
        # gui
        "gui", "gui.canvas", "gui.plot", "gui.renderer", "gui.viewer",
        "gui.widgets", "gui.widgets.TKwidgets", #"gui.3DArray",
        # plugins
        "plugins", "plugins.brains", "plugins.brains.Learning",
        "plugins.devices", "plugins.robots",
        # system
        "system", "system.serial",
        # miscellaneous
        "ai", "aima", "engine", "general", "geometry", "map", "tools",
        ], # </packages>

    # package data
    package_data = {
        'pyrobot.plugins': [
            'simulators/*Simulator', 'simulators/PlayerServer',
            #'worlds/Aria/*', 'worlds/Gazebo/*', 'worlds/Khepera/*',
            #'worlds/Robocup/*', 'worlds/Stage/*', 'worlds/Pyrobot/*',
            'worlds/*/*.world', 'worlds/Pyrobot/*.py',
                        ]
        },

    # extension modules
    ext_modules = [
        AutoIncludeExtension("pyrobot.brain.psom._csom",
            ["brain/psom/csom_src/som_pak.i"] +
            ["brain/psom/csom_src/som_pak-dev/%s" % f for f in
             "datafile.c", "lvq_pak.c", "fileio.c", "labels.c",
             "version.c", "som_devrobs.c", "som_rout.c"]),

        AutoIncludeExtension("pyrobot.vision.cvision._vision",
            ["vision/cvision/%s" % f for f in "Vision.i", "Vision.cpp"],
            include_dirs = ["camera/device"]),

        AutoIncludeExtension("pyrobot.vision.example._myvision",
            ["vision/example/%s" % f for f in "myVision.i", "myVision.cpp"],
            include_dirs = ["camera/device"]),

        AutoIncludeExtension("pyrobot.camera.device._devices",
            ["camera/%s" % f for f in
             "device/Device.i", "device/Device.cpp",
             "fake/Fake.cpp",
             "blob/Blob.cpp",
             "robocup/Robocup.cpp",
             "aibo/Aibo.cpp",
             "aibo/RWLock.cpp",
             "aibo/Socket.cpp",
             "aibo/jpeg.c",
            # <-- add other camera device source files here -->
            ],
            include_dirs=include_dirs,
            library_dirs=library_dirs,
            libraries=["jpeg"]),
        ], # </ext_modules>

    cmdclass = {
        'build_noswig' : BuildNoSwig,
        'build_ext_no_swig': BuildExtensionNoSwig,
        'build_ext': BuildExtension,
        },
    )

import string
import sys
from pyro.system.version import version

def about():
    print "-------------------------------------------------------------"
    print "Pyro - Python Robotics"
    print "Version:", version()
    print "See: http://PyroRobotics.org"
    print "(c) 2004, D.S. Blank"
    print "-------------------------------------------------------------"

def usage():
    print "-------------------------------------------------------------"
    print " Pyro Command Line Arguments:"
    print "-------------------------------------------------------------"
    print "  -h                 show this help"
    print "  -r ROBOT           name of robot.py file to load"
    print "  -b BRAIN           name of brain.py file to load"
    print "  -a ARGUMENTS       user args; available as string engine.args"
    print "  -s SIMULATOR       name of simulator to run"
    print "  -i INI_FILE        name of init file to load"
    print "  -w WORLDFILE       name of simulator world to load"
    print "  -c CONFIGFILE      name of server config to load"
    print "  -d DEVICE[,...]    names of devices (files or names)"
    print "  -e \"string\"        eval string of commands"
    print "                     that are ; separated"    
    print ""

def help():
    print "-------------------------------------------------------------"
    print "Pyro GUI Command line editing commands:"
    print "-------------------------------------------------------------"
    print "  Control+p or UpArrow        previous line"
    print "  Control+n or DownArrow      next line"
    print "  Control+a or Home           beginning of line"
    print "  Control+e or End            end of line"
    print "  Control+f or RightArrow     forward one character"
    print "  Control+b or LeftArrow      back one character"
    print "-------------------------------------------------------------"
    print "Pyro TTY GUI commands:"
    print "-------------------------------------------------------------"
    print "  <command>                   execute <command> in Python"
    print "  <exp>                       print <exp> in Python"
    print "  % <command>                 execute <command> in shell"
    print "  edit                        edit the brain file"
    print "  help                        this help message"
    print "  info                        show brain and robot info"
    print "  load brain                  load a brain file"
    print "  load robot                  load a robot file"
    print "  load simulator              load a simulator"
    print "  quit | exit | bye           exit from Pyro"
    print "  reload                      reload the brain"
    print "  run                         start brain running"
    print "  stop                        stop brain and robot"
    print ""

def file_exists(file_name):
    from posixpath import exists
    if type(file_name) == type(""):
        if len(file_name) == 0:
            return 0
        else:
            return exists(file_name)
    else:
        raise AttributeError, "filename nust be a string"
    
def loadINIT(filename, engine=0, redo=0, brain=0, args=None):
    path = filename.split("/")
    modulefile = path.pop() # module name
    module = modulefile.split(".")[0]
    search = string.join(path, "/")
    oldpath = sys.path[:] # copy
    sys.path.insert(0, search)
    print "Attempting to import '%s'..." % module 
    exec("import " + module + " as userspace")
    reload(userspace)
    print "Loaded '%s'!" % userspace.__file__
    sys.path = oldpath
    try:
        userspace.INIT
    except AttributeError:
        raise ImportError, "your program needs an INIT() function"
    if brain is 0:
        if engine is 0:
            retval = userspace.INIT()
            return retval
        else:
            if args:
                retval = userspace.INIT(engine, args)
                return retval
            else:
                retval = userspace.INIT(engine)
                return retval
    else:
        retval = userspace.INIT(engine, brain)
        return retval

# Module for wrapping command-line business
# Module pyro.tools

import getopt, sys, string

class Option:
    def __init__(self, name, shortForm, argCount, evalFunc, defaultValue,
                 helpText = None):
        self.name = name
        self.shortForm = shortForm
        self.argCount = argCount
        self.evalFunc = evalFunc
        self.defaultValue = defaultValue
        self.helpText = helpText

class OptionHandler:
    def __init__(self, commandName, options = []):
        self.commandName = commandName
        self.options = options

    def __getitem__(self, name):
        return [x for x in self.options if x.name == name][0].value

    def get(self, name):
        return [x for x in self.options if x.name == name][0]

    def makeShortForm(self):
        return string.join([x.shortForm for x in self.options if x.shortForm != "" and x.argCount == 1], ":") + ":" + \
               string.join([x.shortForm for x in self.options if x.shortForm != "" and x.argCount == 0], "")

    def makeLongForm(self):
        return [x.name + "=" for x in self.options if x.argCount == 1] + [x.name for x in self.options if x.argCount == 0]

    def help(self):
        print '%s Options:' % self.commandName
        print '----------------'
        for opt in self.options:
            print '      --%-15s\t-%s\tDefault value: %s\t\t%s' % (opt.name, opt.shortForm, opt.defaultValue, opt.helpText)

    def initialize(self, commandLine):
        shortforms = self.makeShortForm()
        longforms = self.makeLongForm()
        try:
            opts, args = getopt.getopt(commandLine, shortforms, longforms)
        except getopt.GetoptError:
            self.help()
            sys.exit(1)
        # first, go through and define default values:
        for command in self.options:
            if command.argCount == 1:
                command.value = command.evalFunc(command.defaultValue)
        # now, overwrite those that came in on the commandline:
        for (o, a) in opts:
            ok = 0
            for command in options:
                if o in ("--" + command.name, "-" + command.shortForm):
                    command.value = command.evalFunc(a)
                    ok = 1
                    break
            if not ok:
                self.help()
                sys.exit(1)
        return args

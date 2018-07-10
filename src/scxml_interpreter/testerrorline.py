from scxml_interpreter.Errorexecptions import *
import traceback

class Error(Exception):
    """Base class for exceptions in this module."""
    pass
def my_exec(cmd, globals=None, locals=None, description='source string'):
    try:
        exec(cmd, globals, locals)
    except InterpreterError as err:
        detail = err.args[0]
        line_number = err.lineno
    except Exception as err:
        detail = err.args[0]
        cl, exc, tb = sys.exc_info()
        line_number = traceback.extract_tb(tb)[-1][1]
        print line_number
    else:
        return
    raise Error("At line %d of %s: %s" % (line_number, description, detail))

if __name__=="__main__":
    test=my_exec("1+1\n'''")
    print test

import sys
import termios
import tty

class ReadFromStd:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def get_float(self):
        num = self.get_string()
        try:
            num = float(num)
        except ValueError:
            raise ValueError('Expected valid float input, got "%s".' % num)
        
        return num

    def get_string(self):
        string = ""
        while True:
            ch = self()
            if ch == '\r':
                break

            if ch == '\x7f':
                if string != "":
                    string = string[:-1]
                    print('\b \b', end='', flush=True)
            else:
                string += ch
                print(ch, end='', flush=True)
        
        print('')
        return string

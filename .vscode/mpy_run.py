import codecs
import sys
import mpremote.transport

sys.stdout.reconfigure(encoding="utf-8")

_decoder = codecs.getincrementaldecoder("utf-8")()

def _utf8_write_bytes(b):
    b = b.replace(b"\x04", b"")
    text = _decoder.decode(b, final=False)
    sys.stdout.write(text)
    sys.stdout.flush()

mpremote.transport.stdout_write_bytes = _utf8_write_bytes

from mpremote import main
main.main()

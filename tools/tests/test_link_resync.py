"""Review fix H: Link must give every command its own reply when the device sends a late OK
after a timeout or a stray ERR 0C/0D (spec 2.1: replies are in command order)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import recorder as rc  # noqa: E402

PING_OK = "OK PING ver=02 bld=20261201 cal=1 uok=1"


class ScriptTransport:
    """Instant fake line transport. handler(line) -> list of reply lines (queued FIFO).
    readline() pops the queue, returns None when empty (= timeout elapsed)."""

    def __init__(self, handler):
        self.handler, self.q, self.sent = handler, [], []

    def write(self, data):
        for ln in data.decode("ascii").replace("\r", "\n").split("\n"):
            if ln.strip():
                self.sent.append(ln)
                self.q.extend(self.handler(ln))

    def readline(self, timeout):
        return self.q.pop(0) if self.q else None


def dev(extra=None):
    """Well-behaved device; extra: {command line: [reply lines]} overrides."""
    extra = extra or {}

    def h(line):
        if line in extra:
            return list(extra[line])
        return {"PING": [PING_OK], "MODE BP": ["OK MODE bp=1"], "TEL 0064": ["OK TEL tel=0064"],
                "STOP": ["OK STOP"], "LEN 00000020": ["OK LEN len=00000020"]}.get(line, ["ERR 02"])
    return h


class TestLinkResync(unittest.TestCase):
    def test_late_ok_after_timeout(self):
        tp = ScriptTransport(dev({"TEL 0064": []}))      # device silent -> timeout
        link = rc.Link(tp, timeout=0.01)
        with self.assertRaises(TimeoutError):
            link.command("TEL 0064")
        tp.q.append("OK TEL tel=0064")                   # the late reply arrives afterwards
        r = link.command("MODE BP")
        self.assertEqual(r.msg.name, "MODE")
        self.assertEqual(r.msg["bp"], 1)
        self.assertIn("PING", tp.sent)                   # resync happened
        r = link.command("LEN 00000020")                 # and stays in step
        self.assertEqual(r.msg.name, "LEN")

    def test_late_ok_same_command_after_timeout(self):
        tp = ScriptTransport(dev({"PING": []}))
        link = rc.Link(tp, timeout=0.01)
        with self.assertRaises(TimeoutError):
            link.command("PING")
        tp.handler = dev()
        tp.q.append(PING_OK)                             # late OK PING of the lost PING
        r = link.command("MODE BP")
        self.assertEqual(r.msg.name, "MODE")

    def test_stray_err_before_command(self):
        tp = ScriptTransport(dev())
        link = rc.Link(tp)
        tp.q.append("ERR 0C")                            # unsolicited, already in the buffer
        r = link.command("MODE BP")
        self.assertEqual(r.msg.name, "MODE")

    def test_stray_err_0c_between_command_and_reply(self):
        tp = ScriptTransport(dev({"MODE BP": ["ERR 0C", "OK MODE bp=1"]}))
        link = rc.Link(tp)
        r = link.command("MODE BP")
        self.assertTrue(r.msg.kind == "OK" and r.msg.name == "MODE")
        tp.handler = dev({"MODE BP": ["OK MODE bp=1", "ERR 0D"]})   # ERR trails the OK
        link.command("MODE BP")
        r = link.command("STOP")
        self.assertEqual(r.msg.name, "STOP")

    def test_stale_ok_of_other_command(self):
        tp = ScriptTransport(dev({"MODE BP": ["OK STOP", "OK MODE bp=1"]}))
        r = rc.Link(tp).command("MODE BP")
        self.assertEqual(r.msg.name, "MODE")

    def test_real_err_is_still_the_reply(self):
        tp = ScriptTransport(dev({"MODE BP": ["ERR 04"]}))
        link = rc.Link(tp)
        with self.assertRaises(rc.DeviceError) as cm:
            link.command("MODE BP")
        self.assertEqual(cm.exception.code, 4)
        tp.handler = dev()
        self.assertEqual(link.command("MODE BP").msg.name, "MODE")

    def test_leading_eol_once_and_async_kept(self):
        tp = ScriptTransport(dev({"STOP": ["!S st=0 md=0 t=00000010", "OK STOP"]}))
        writes = []
        orig = tp.write
        tp.write = lambda d: (writes.append(d), orig(d))[1]
        link = rc.Link(tp)
        link.command("MODE BP")
        r = link.command("STOP")
        self.assertEqual(writes[0], b"\n")
        self.assertEqual(sum(1 for w in writes if w == b"\n"), 1)
        self.assertEqual([m.name for m in r.pre], ["S"])
        self.assertEqual(link.async_log[-1].name, "S")


if __name__ == "__main__":
    unittest.main()

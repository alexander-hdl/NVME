#!/usr/bin/env python3
"""Decode fpga/build/ila_capture.csv (ILA u_ila of ax7a200b_top, probe = nvme_host_top.dbg_uc[303:0]).

Prints every cycle in which something moves on the pcie_7x AXI-Stream interface (TX or RX beat accepted), plus
changes of the sequencer state / reset / link / tx_buf_av.  The 128-bit beats are shown as 4 DWs (DW0 = [31:0],
the 7-series core puts the first header DW there).  For a SOF the TLP type is decoded from that DW.

    python fpga/tools/ila_decode.py [fpga/build/ila_capture.csv] [--all]
"""
import csv
import sys

SEQ = ['IDLE', 'LINK', 'ENUM', 'RPCFG', 'ADMIN_INIT', 'RUN', 'DRAIN', 'FLUSH', 'SHUTDOWN', 'DONE', 's10',
       's11', 's12', 's13', 's14', 'ERROR']


def fld(v, lo, w):
    return (v >> lo) & ((1 << w) - 1)


def tlp_name(h0):
    fmt, typ = fld(h0, 29, 3), fld(h0, 24, 5)
    ln = fld(h0, 0, 10) or 1024
    names = {(0, 0): 'MRd32', (1, 0): 'MRd64', (2, 0): 'MWr32', (3, 0): 'MWr64', (0, 4): 'CfgRd0', (2, 4): 'CfgWr0',
             (0, 5): 'CfgRd1', (2, 5): 'CfgWr1', (0, 0xA): 'Cpl', (2, 0xA): 'CplD', (1, 0x10): 'Msg', (3, 0x10): 'MsgD'}
    n = names.get((fmt, typ))
    if n is None and fmt in (1, 3) and typ & 0x18 == 0x10:
        n = 'Msg' + ('D' if fmt == 3 else '')
    return f"{n or f'fmt{fmt}/type{typ:02x}'} len={ln}"


def main():
    path = 'fpga/build/ila_capture.csv'
    show_all = '--all' in sys.argv
    for a in sys.argv[1:]:
        if not a.startswith('--'):
            path = a
    with open(path, newline='') as f:
        rows = list(csv.reader(f))
    hdr = rows[0]
    col = next(i for i, h in enumerate(hdr) if 'dbg_uc' in h or 'probe0' in h)
    trig_col = next((i for i, h in enumerate(hdr) if h.strip().upper() == 'TRIGGER'), None)
    prev_ctl = None
    for r in rows[1:]:
        if not r or not r[0].strip().isdigit():
            continue                                   # radix line etc.
        v = int(r[col], 16)
        smp = int(r[0])
        seq, urst, lnk = fld(v, 0, 4), fld(v, 4, 1), fld(v, 5, 1)
        txv, txr, txl, rxv, rxr, rxl = (fld(v, b, 1) for b in range(6, 12))
        bufav = fld(v, 12, 6)
        cfg = fld(v, 18, 3)
        npok = fld(v, 21, 1)
        txd, rxd, rxu = fld(v, 24, 128), fld(v, 152, 128), fld(v, 280, 22)
        ctl = (seq, urst, lnk, bufav, cfg, npok)
        trig = (trig_col is not None and r[trig_col].strip() == '1')
        notes = []
        if ctl != prev_ctl or show_all or trig:
            notes.append(f"seq={SEQ[seq]} user_reset={urst} lnk_up={lnk} tx_buf_av={bufav} cfg_mgmt(rd,wr,done)="
                         f"{cfg & 1}{(cfg >> 1) & 1}{(cfg >> 2) & 1} rx_np_ok={npok}")
            prev_ctl = ctl
        if txv:
            dws = [fld(txd, 32 * i, 32) for i in range(4)]
            notes.append(f"TX{'*' if txr else ' (not ready)'} last={txl} DW0..3 = " + ' '.join(f"{d:08x}" for d in dws)
                         + f"   [{tlp_name(dws[0])}?]")
        if rxv:
            dws = [fld(rxd, 32 * i, 32) for i in range(4)]
            sof = fld(rxu, 14, 1)
            sofpos = fld(rxu, 10, 4)
            eof = fld(rxu, 21, 1)
            eofpos = fld(rxu, 17, 4)
            t = ''
            if sof:
                t = f"  SOF@DW{sofpos >> 2 if sofpos else 0}: {tlp_name(dws[(sofpos >> 2) & 3])}"
            notes.append(f"RX{'*' if rxr else ' (not ready)'} sof={sof} eof={eof}(pos {eofpos}) DW0..3 = "
                         + ' '.join(f"{d:08x}" for d in dws) + t)
        if notes:
            print(f"{smp:5d}{' T' if trig else '  '} " + ' | '.join(notes))


if __name__ == '__main__':
    main()

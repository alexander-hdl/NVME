"""was_done.md -> was_done.pdf (A4) via python-markdown + headless Edge.

usage: python tools/md2pdf.py [in.md] [out.pdf]
Figures from docs/10_slc_cache.html (Fig. 10.2, 10.4, 10.5) are inserted after subsections 10.3 and 10.5 and under the 10.9 heading.
"""
import os, re, subprocess, sys, tempfile, time
import markdown

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'was_done.md')
dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + '.pdf'
EDGE = r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'

md = open(src, encoding='utf-8').read()

# python-markdown is stricter than GitHub: a list needs a blank line before it, nesting needs 4 spaces
LIST = re.compile(r'^(\s*)([*+-]|\d+\.)\s')
out, in_code = [], False
for line in md.split('\n'):
    if line.startswith('```'):
        in_code = not in_code
    if not in_code:
        m = LIST.match(line)
        if m:
            line = ' ' * (len(m.group(1)) * 2) + line[len(m.group(1)):]
            prev = out[-1] if out else ''
            if prev.strip() and not LIST.match(prev) and not prev.startswith('|'):
                out.append('')
    out.append(line)
md = '\n'.join(out)
body = markdown.markdown(md, extensions=['tables', 'fenced_code', 'sane_lists'])

# figures from the textbook chapter
ch = open(os.path.join(ROOT, 'docs', '10_slc_cache.html'), encoding='utf-8').read()
def fig(label, caption):
    m = re.search(r'<svg[^>]*aria-label="' + label + r'.*?</svg>', ch, re.S)
    return f'<figure>{m.group(0)}<figcaption>{caption}</figcaption></figure>' if m else ''
body = body.replace('<h3>10.4.', fig('Скорость записи по мере',
    'Рис. 1. Скорость записи по мере заполнения диска (интервалы 10 с, лог. шкала).') + '<h3>10.4.', 1)
body = re.sub(r'(<h3>10\.9\..*?</h3>)', lambda m: m.group(1) + fig('Профили записи всего диска',
    'Рис. 3. Запись всего диска на полной скорости: три SSD (интервалы 10 с, лог. шкала).'), body, count=1)
body = body.replace('<h3>10.6.', fig('Потребная память',
    'Рис. 2. Память для записи без потерь в зависимости от скорости источника. Зелёная полоса — BRAM XC7A200T.') + '<h3>10.6.', 1)

css = """
:root{--fg:#1d2430;--muted:#5b6675;--accent:#1f5fbf;--accent2:#b8431f;--code-bg:#f4f6f9;--border:#c9d0da;--ok-bg:#eefaf0}
@page{size:A4;margin:16mm 14mm 16mm 14mm}
body{font:10pt/1.45 "Segoe UI",Arial,sans-serif;color:var(--fg);margin:0}
h1{font-size:18pt;margin:0 0 6pt}
h2{font-size:14pt;margin:16pt 0 6pt;padding-top:4pt;border-top:1px solid var(--border);break-after:avoid}
h3{font-size:11.5pt;margin:12pt 0 4pt;break-after:avoid}
p,li{orphans:3;widows:3}
code{font-family:Consolas,"Cascadia Mono",monospace;font-size:8.8pt;background:var(--code-bg);padding:0 3px;border-radius:3px}
pre{background:var(--code-bg);border:1px solid var(--border);border-radius:6px;padding:6pt 8pt;font-size:8pt;line-height:1.25;white-space:pre;overflow:hidden;break-inside:avoid}
pre code{background:none;padding:0;font-size:8pt}
table{border-collapse:collapse;width:100%;margin:6pt 0;font-size:8.8pt;break-inside:auto}
tr{break-inside:avoid}
th,td{border:1px solid var(--border);padding:3pt 5pt;vertical-align:top;text-align:left}
th{background:var(--code-bg)}
figure{margin:8pt 0;text-align:center;break-inside:avoid}
figure svg{width:100%;height:auto}
figcaption{font-size:8.5pt;color:var(--muted)}
hr{border:0;border-top:1px solid var(--border)}
a{color:var(--accent);text-decoration:none}
"""
html = f'<!doctype html><html lang="ru"><head><meta charset="utf-8"><title>Отчёт о проделанной работе</title><style>{css}</style></head><body>{body}</body></html>'

with tempfile.TemporaryDirectory() as td:
    hp = os.path.join(td, 'report.html')
    open(hp, 'w', encoding='utf-8').write(html)
    if os.path.exists(dst):
        os.remove(dst)
    subprocess.run([EDGE, '--headless=new', '--disable-gpu', '--no-pdf-header-footer',
                    f'--user-data-dir={os.path.join(td, "prof")}', f'--print-to-pdf={os.path.abspath(dst)}',
                    'file:///' + hp.replace('\\', '/')], check=True, timeout=120)
    # Edge may hand the job to a background process and return early: wait for the file to settle
    last = -1
    for _ in range(120):
        size = os.path.getsize(dst) if os.path.exists(dst) else -1
        if size > 0 and size == last:
            break
        last = size
        time.sleep(1)
print(dst, os.path.getsize(dst))

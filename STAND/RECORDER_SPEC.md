# Регистратор на AX7A200B: спецификация и архитектура прототипа

Версия 0.2, 2026-11-24. Статус: **утверждено к кодированию** (решения оркестратора по ревью `STAND/RECORDER_SPEC_REVIEW.md`).
Основа: ядро `nvme_host_top` (ARCHITECTURE.md), стенд `STAND/AX7A200B_ARCHITECTURE.md`, измерения `was_done.md` раздел 10.
Язык RTL — SystemVerilog-2012, Vivado 2025.2, xsim (`sim/scripts/run_xsim.sh <name>`). Правила кода — `CODING_RULES.md`, журнал стыков — `INTEGRATION_NOTES.md`.

## Журнал изменений

| Версия | Изменение (ссылка на ревью) |
|---|---|
| 0.2 | **B1**: в MIG XADC Instantiation = Disabled, `device_temp_i` от `xadc_mon` (переход X14) |
| | **I1**: два вида очистки: безусловный `ui_clear` при каждом старте; сброс ядра (состояние `CORE_RST`) после любого сеанса с ошибкой и после `ABORT` |
| | **I2**: база `beats_in` защёлкивается при входе в RUN; используются уже существующие `stat_beats_in`, `status_seq_state` (только проброс в `nvme_host_top`) |
| | **I3/M14**: переходы SRC_STOP→CORE_STOP по уходу ядра в FLUSH/SHUTDOWN/DONE; PF из SRC_STOP/CORE_STOP |
| | **I4/M1/M2**: путь `in_log` для всех четырёх точек `do_fail`, `S_LOG_EX`, поглощение запоздалого CQE только по совпадению CID, условия приёма `log_req`, RTD3E — отдельный шаг разбора; `log_req` не выдаётся после STOP/PF |
| | **I5**: тесты ≥ 100 логов подряд (2+ оборота admin-очереди) при doorbell порта 1 |
| | **I6/M7**: `set_bus_skew` на gray-указатели и `cdc_bus`; группа `free_clk -include_generated_clocks` против PCIe |
| | **I7/I8**: контракт сброса примитивов CDC, «возраст» снимков; раздельные сбросы регистратора и ядра |
| | **I9**: сверка перенесена в `ui_clk`: `vrf_tap` (user_clk, только регистр) → async FIFO → `rec_verify` (ui_clk) |
| | **I10**: протокол — текст, но все числа hex фиксированной ширины в обе стороны, без суффиксов; единицы, десятичные и пересчёт — в `recorder.py`. `seq_mul_div`, `bin2dec` удалены |
| | **I11**: `smart_t` в `rec_pkg` (WP0), ядро выдаёт плоский вектор `log_smart[383:0]` с раскладкой из 6.3; Tcl MIG принадлежит только WP4 |
| | **I12**: системный тест с неровным соотношением тактов и дрожанием фазы |
| | **M3/M4/M6/M9–M13/M15/M17**: `verify_cmds` для READ END = ёмкость; ERROR снимает `ctrl_start`; `step` отдельной шиной, `len_blocks` с `src_start`; `UI_CLK_HZ` в `rec_pkg`; DDR только целыми пачками 2 КиБ; запас `space_ok`; точное условие конца слива; PAD увеличивает `seq` и `pos` |
| | **Вырезано из v1**: XADC min/max, `FMT RAW`, запасные 333 МГц, запись неполной пачки по тайм-ауту, `PROF END ZERO`; профиль 64 сегмента; оценка мощности — только на ПК (0 логики FPGA) |
| | `.prj` MIG пишется как XML автором WP4 (без GUI), проверка `create_ip` + сверка выводов в batch |
| | Выход `ssd_pwr_en` назначен на J11-3 (G21); вход PF в v1 — только KEY3 и команда |
| 0.1 | первая редакция |

Принципы:

* Ядро NVMe (`rtl/nvme/*`) меняется минимально и только в местах из таблицы 4.6.
* Старый топ `rtl/board/ax7a200b_top.sv` остаётся (регрессионный битстрим). Регистратор — новый топ `rtl/board/ax7a200b_rec_top.sv`.
* Новая логика — в `rtl/rec/` и `rtl/common/`; каждый файл принадлежит одному пакету работ (раздел 10).
* В домен `user_clk` (WNS +0,062 нс) не добавляется ничего, кроме регистра tap и логики лога в `nvme_admin`.
* FPGA не делает арифметики с единицами: только hex-поля и сырые счётчики. Всё человеческое — в `tools/recorder.py`.

---

## 1. Назначение, режимы, автомат контроллера

### 1.1 Что делает прототип

| Функция | Как |
|---|---|
| Управление с ПК | UART 921600 8N1, текстовые команды с hex-числами (раздел 2), утилита `tools/recorder.py` |
| Источник данных | генератор с заголовками блоков (раздел 3): постоянная скорость или профиль до 64 сегментов, переключение точно в такт `ui_clk` |
| Буфер | DDR3 модуля (1 ГиБ, MIG) как FIFO: генератор → DDR3 → кольцо BRAM → NVMe |
| Потери | счётчик выброшенных блоков, заполнение DDR (текущее/максимум); потеря видна и в данных на SSD |
| Отключение питания (эмуляция) | команда или KEY3: стоп источника → дописать DDR и кольцо → Flush → CC.SHN; время фаз и объём «в пути» |
| Чтение и сверка | Read записанного диапазона (или до конца валидных блоков), проверка заголовков, CRC и шаблона в FPGA (`ui_clk`) |
| Телеметрия | SMART/Health (Log 02h) во время сеанса; XADC (температура, VCCINT/AUX/BRAM); оценка мощности FPGA — на ПК |

Запись и чтение никогда не идут одновременно: один сеанс ядра = один режим.

### 1.2 Режимы (`md`, код hex)

| Код | Режим | Команда | Сеанс ядра | Источник | DDR (`calib` нужна?) |
|---|---|---|---|---|---|
| 0 | — | — | нет | стоп | — |
| 1 | REC | `REC`, `REC PROF` | LINK…RUN→DRAIN→FLUSH→SHUTDOWN→DONE | вкл. с входа в RUN | FIFO (да) |
| 2 | PF | `PFAIL`, KEY3 в REC | тот же сеанс, конец по PF | стоп в такт события | дописывается |
| 3 | READ | `READ …` | тот же, `verify_mode=1` | выкл. | не используется, но сверка в `ui_clk` (да) |
| 4 | FORMAT | `FORMAT YES` | `format_req=1`, STOP сразу после RUN | выкл. | нет |
| 5 | MON | `MON` | сеанс без данных, SMART до `STOP` | выкл. | нет |
| 6 | DTEST | `DTEST` | нет | вкл. | FIFO → `rec_sink_chk` (да) |
| 7 | MEMTEST | `MEMTEST [n]` | нет | выкл. | тест памяти (да) |

### 1.3 Автомат `rec_ctrl` (домен `link_clk`)

Состояния (`st`, hex): 0 `IDLE`, 1 `BRINGUP`, 2 `RUN`, 3 `SRC_STOP`, 4 `CORE_STOP`, 5 `RELEASE`, 6 `LOCAL`, 7 `ERROR`, 8 `CORE_RST`.

Обозначения: `cs` — состояние `host_sequencer` (0 IDLE … 5 RUN, 6 DRAIN, 7 FLUSH, 8 SHUTDOWN, 9 DONE, F ERROR) из свежего снимка `core_stat` (4.3, правило «возраста»); `ui_clear` — импульс X1: обнуление `rec_profile`, `rec_framer`, `ddr_fifo` (с ожиданием ответов MIG в полёте), `rec_verify`, `rec_sink_chk`.

| Из | Условие | В | Действия |
|---|---|---|---|
| IDLE | `REC`/`READ`/`FORMAT YES`/`MON` | BRINGUP | `ui_clear`; защёлкнуть конфигурацию; `ctrl_start=1`; REC/READ требуют `calib` и `ui_ok` |
| IDLE | `MEMTEST`, `DTEST` | LOCAL | `ui_clear`; старт теста |
| BRINGUP | `cs=RUN` | RUN | защёлкнуть `beats_base = beats_in`; REC: `src_start` (вместе с `len_blocks`, 4.3 X2b); FORMAT: сразу `ctrl_stop=1` → CORE_STOP; MON/READ: ждать |
| BRINGUP | `STOP` или PF | CORE_STOP | `ctrl_stop=1`; PF: `t0`, `inBringup=1` |
| RUN (REC) | `STOP`, `len_reached`, `prof_end`, ёмкость | SRC_STOP | `src_stop` |
| RUN (REC) | PF | SRC_STOP | `src_pf`, `t0` |
| RUN, SRC_STOP | `cs ∈ {FLUSH, SHUTDOWN, DONE}` (ядро остановилось само: io_timeout, lba_range_err) | CORE_STOP | `src_stop` (если не было); `ctrl_stop=1`; сеанс помечается `errored` |
| RUN (READ) | `cmd_limit_done`, `end_found`, `STOP`, PF | CORE_STOP | `ctrl_stop=1` |
| RUN (MON) | `STOP`, PF | CORE_STOP | `ctrl_stop=1` |
| SRC_STOP | `src_done && ddr_idle && (beats_in − beats_base) == wr_blocks×256` (оба снимка свежие) | CORE_STOP | `ctrl_stop=1` |
| SRC_STOP, CORE_STOP | PF (после STOP) | то же | `t0` = момент PF, флаг `pfLate=1`; фазы считаются от `t0` |
| CORE_STOP | `cs=DONE` | RELEASE | защёлкнуть итоги, `!RES …`; `ctrl_start=0`; `errored |= (ce≠0) or fl[0,2,3]` |
| RELEASE | `cs=IDLE` | IDLE или CORE_RST | `ctrl_stop=0`; если `errored` → CORE_RST |
| BRINGUP…CORE_STOP | `cs=ERROR` | ERROR | `src_stop`, `ctrl_start=0`, `ctrl_stop=0`; `!ERR` |
| ERROR | сразу | CORE_RST | |
| LOCAL | тест окончен / `STOP` | IDLE | `!RES MEMTEST`/`!RES DTEST` |
| любое | `ABORT` | CORE_RST | `ctrl_start=0`, `ctrl_stop=0`, `src_stop`; SSD не выключается штатно |
| CORE_RST | — | IDLE | `core_rst_req=1` на 1 мс (сбрасывает ядро, `data_ring`, `link_lost`); `ui_clear`; ждать свежий снимок с `cs=IDLE` ≤ 2 с, иначе `!ERR` и повтор |

Правила: `log_req` выдаётся только в RUN и только до STOP/PF (M2). Сброс регистратора (`rec_rst`: включение и кнопка RESET) и сброс ядра (`core_rst_n`) раздельны (4.2).

---

## 2. UART-протокол

### 2.1 Физика и общие правила

| Параметр | Значение |
|---|---|
| Линия | CP2102GM: FPGA `uart_rx` = L14 (UART1_RXD), `uart_tx` = L15 (UART1_TXD), LVCMOS33 |
| Формат | 921600 бод, 8N1, без управления потоком; делитель 200 МГц / 921600 = 217 (ошибка 0,006 %) |
| Строка команды | ASCII, конец — CR, LF или CRLF; ≤ 64 символа; регистр не важен; эха нет |
| Числа | **только hex** без префикса. Вход: 1…16 цифр (ведущие нули можно опускать; `recorder.py` всегда шлёт фиксированную ширину). Выход: всегда фиксированная ширина по таблицам ниже |
| Ответ | на каждую команду ровно одна строка `OK …` или `ERR …` (у `PROF?` перед `OK` идут строки данных) |
| Асинхронные строки | `$T …` (телеметрия), `!…` (события). Никогда не врезаются внутрь другой строки |
| Конвейер | ПК обязан дождаться `OK/ERR` (после `PROF?` — после `OK`) и только потом слать следующую строку; ответы идут строго в порядке команд. `ERR 0C` — новая строка пришла, пока предыдущая команда ещё не принята контроллером; после приёма ответ на вторую строку тоже ставится в очередь, поэтому `ERR 0C`/`ERR 0D` могут прийти без своей команды — `recorder.py` сверяет ключевое слово `OK` с командой, считает `ERR` ответом, только если за ним не следует подходящий `OK`, и после тайм-аута ресинхронизируется по `PING` |
| Поля | `ключ=значение` через пробел; ключи — нижний регистр, значения — hex верхнего регистра |

Обозначение ширины: `h2` = 2 hex-цифры (u8), `h4` (u16), `h8` (u32), `h10` (u40), `h12` (u48), `h16` (u64).

### 2.2 Грамматика

```
line    := WS* keyword (WS+ arg){0,3} WS* EOL
keyword := [A-Za-z?]{1,8}
arg     := word | hex
word    := [A-Za-z]{1,8}          ; подкоманда: PROF, ADD, SET, REP, END, HOLD, STOP, LAST, SID, YES, RT, BP
hex     := [0-9A-Fa-f]{1,16}      ; слово из одних hex-букв (ADD, BEEF) трактуется как word, если такое слово допустимо в этой позиции
```

### 2.3 Команды

Пересчёт единиц (делает `recorder.py`, константы в `rec_pkg`): `UI_CLK_HZ = 100 000 000`; шаг NCO `step = round(R[B/s] × 2^32 / (16 × UI_CLK_HZ))`, фактическая `R = step × 16 × UI_CLK_HZ / 2^32`; длительность `dur = мкс × 100` тактов; `LEN`/`READ` — в слотах по 128 КиБ (вверх); `DDRSZ` — в МиБ.

| Команда | Аргументы | Состояние | Ответ |
|---|---|---|---|
| `PING` | — | любое | `OK PING ver=h2 bld=h8 cal=h1 uok=h1` (`bld` = yyyymmdd как hex-цифры) |
| `STAT?` | — | любое | строка `$T`, затем `OK` |
| `CFG?` | — | любое | `OK CFG md=h1 bp=h1 stp=h8 len=h8 sid=h8 ddr=h4 tel=h4 sm=h4 pn=h2 pr=h4 pe=h1 rsid=h8` |
| `RES?` | — | любое | повтор последней `!R…`, затем `OK` |
| `TEL <ms:h4>` | 0 или ≥ 000A | любое | `OK TEL tel=h4`; 0 = выкл.; по умолчанию 03E8 (1 с) |
| `SMART <ms:h4>` | 0 или ≥ 0064 | любое | `OK SMART sm=h4`; по умолчанию 03E8 |
| `SMART?` | — | любое | `OK SMART cw=h2 tk=h4 sp=h2 us=h2 dur=h16 duw=h16 pc=h8 poh=h8 uns=h8 me=h8 wt=h8 ct=h8 t1=h4 ok=h4 er=h4 st=h4 rtd=h8` |
| `MODE RT\|BP` | слово | IDLE | `OK MODE bp=h1`; RT — реальное время (выброс блоков), BP — источник ждёт (без потерь, для измерений как в 10.4) |
| `RATE <step:h8>` | — | IDLE, RUN (REC без профиля), LOCAL (DTEST) | `OK RATE stp=h8`; в RUN применяется ≤ 20 тактов `ui_clk` |
| `LEN <slots:h8>` | 0 = до STOP/ёмкости | IDLE | `OK LEN len=h8` |
| `SID <h8>` | — | IDLE | `OK SID sid=h8`; без `SID` — счётчик +1 на каждый REC |
| `DDRSZ <mib:h4>` | 0001…0400 | IDLE | `OK DDRSZ ddr=h4`; по умолчанию 0400 (1 ГиБ) |
| `PROF CLR` | — | IDLE | `OK PROF pn=00` |
| `PROF ADD <step:h8> <dur:h10>` | dur ≥ 1 | IDLE | `OK PROF i=h2`; 65-й → `ERR 09` |
| `PROF SET <i:h2> <step:h8> <dur:h10>` | i < pn | IDLE | `OK PROF i=h2` |
| `PROF REP <n:h4>` | 0 = бесконечно | IDLE | `OK PROF pr=h4` (по умолчанию 0001) |
| `PROF END HOLD\|STOP` | слово | IDLE | `OK PROF pe=h1` (0 HOLD, 1 STOP — по умолчанию) |
| `PROF?` | — | любое | строки `P i=h2 stp=h8 dur=h10`, затем `OK PROF pn=h2 pr=h4 pe=h1` |
| `REC` / `REC PROF` | — | IDLE | `OK REC sid=h8`; пустой профиль → `ERR 08` |
| `STOP` | — | BRINGUP, RUN, LOCAL | `OK STOP` |
| `PFAIL` | — | BRINGUP, RUN, SRC_STOP, CORE_STOP | `OK PFAIL` |
| `READ LAST` | — | IDLE | диапазон последней записи этого включения (`wr_blocks`); нет → `ERR 0B` |
| `READ <slots:h8>` | > 0 | IDLE | столько слотов с LBA 0 |
| `READ END` | — | IDLE | до первого невалидного блока, `verify_cmds = ⌊ёмкость/128 КиБ⌋` (M3) |
| `READ SID <h8>` | — | IDLE | ожидаемый SID следующего READ (`rsid`; по умолчанию SID последней записи) |
| `FORMAT YES` | `YES` обязателен | IDLE | сеанс с Format NVM (SES=0) |
| `MON` | — | IDLE | сеанс без данных |
| `MEMTEST [<mib:h4>]` | по умолчанию `DDRSZ` | IDLE | |
| `DTEST` | — | IDLE | до `STOP` |
| `DDR?` | — | любое | `OK DDR cal=h1 ddr=h8 ddrm=h8 mte=h8 mta=h8` |
| `ABORT` | — | любое | `OK ABORT` (ответ уходит: `rec_ctrl` не сбрасывается) |

### 2.4 Коды ошибок (`ERR <h2>`)

| Код | Когда |
|---|---|
| 01 | синтаксис, лишние/недостающие аргументы |
| 02 | неизвестное ключевое слово |
| 03 | аргумент вне диапазона |
| 04 | недопустимо в текущем состоянии (в т.ч. `RATE` при профиле) |
| 05 | DDR не откалибрована / `ui_clk` не готов |
| 06 | ядро в ошибке, идёт CORE_RST |
| 08 | `REC PROF` с пустым профилем |
| 09 | профиль полон (64) |
| 0A | строка > 64 символов |
| 0B | `READ LAST` без записи |
| 0C | новая строка пришла, пока предыдущая команда ещё не принята контроллером |
| 0D | ошибка стоп-бита UART в строке |

### 2.5 Телеметрия `$T`

Поля в фиксированном порядке, ≈ 290 символов (3,1 мс на линии; при `TEL 000A` — 31 %). Каждое значение — снимок; `cok`/`uok` = свежесть снимков ядра и `ui` (4.3).

| Ключ | Шир. | Ед. | Источник | Смысл |
|---|---|---|---|---|
| `v` | h2 | — | — | версия строки (01) |
| `t` | h8 | мс | link | время от сброса платы |
| `st`, `md` | h1, h1 | код | rec_ctrl | 1.2, 1.3 |
| `cok`, `uok` | h1, h1 | — | 4.3 | снимок ядра / `ui` свежий (< 1 мс) |
| `cs`, `ce`, `cd`, `fl` | h1, h2, h2, h2 | код | ядро | состояние, `status_error`, `status_error_detail`, `status_flags` |
| `lk` | h1 | бит | ядро | b0 link_up, b1 Gen2, b3:2 ширина-1 |
| `cal` | h1 | — | ui | `init_calib_complete` |
| `stp` | h8 | шаг | ui | текущий шаг NCO |
| `seg`, `rep` | h2, h4 | — | ui | сегмент и проход профиля |
| `src`, `wr` | h12, h12 | блоки 4 КиБ | ui | сгенерировано (с выброшенными) / выдано в FIFO (с PAD) |
| `drp` | h8 | блоки | ui | выброшено |
| `ddr`, `ddrm` | h8, h8 | Б | ui | заполнение FIFO (DDR+staging+выход) / максимум сеанса |
| `bpm` | h10 | такты 16 Б | ui | BP: максимальный долг источника |
| `rng` | h8 | такты 16 Б | ядро | `beats_in − beats_base − ssd/16` |
| `ssd` | h16 | Б | ядро | `stat_bytes_done` |
| `cmd`, `cer` | h8, h8 | — | ядро | команды, ошибки |
| `lat` | h8 | такты 8 нс | ядро | `stat_max_latency` |
| `vb`, `ve` | h12, h8 | Б, такты | ui (`rec_verify`) | сверено, ошибочных тактов |
| `tk`, `cw` | h4, h2 | K, — | SMART | Composite Temperature (0 — не читали), Critical Warning |
| `smo`, `sme` | h4, h4 | — | ядро | успешные / ошибочные Get Log Page |
| `xt`, `xvi`, `xva`, `xvb` | h3 ×4 | код 12 бит | XADC | T = код×503,975/4096 − 273,15 °C; U = код×3/4096 В (пересчёт на ПК) |

### 2.6 События

| Строка | Когда | Поля (все hex) |
|---|---|---|
| `!B ver= cal= uok=` | после сброса регистратора и после калибровки | |
| `!S st= md= t=h8` | каждый переход `rec_ctrl` | |
| `!E ce= cd= fl= st=` | ядро в ERROR или сеанс с ошибкой | |
| `!RR` (REC) | конец записи | `sid wr drp pad trc ddrm ssd cmd cer lat dur=h16` (`dur` — такты 5 нс от RUN до DONE) |
| `!RP` (PF) | конец по PF | раздел 7.2 |
| `!RV` (READ) | конец чтения | `vb ok=h10 bad=h8 crc=h8 pe=h8 ooo=h8 prt=h8 fbp=h10 mvp=h10 lsq=h16 lts=h16 lst=h8 pad=h8 fep=h10 feb=h2 exp=h8 got=h8 end=h1 ovf=h1 dur=h16` |
| `!RF` (FORMAT) | | `ms=h8 ok=h1` |
| `!RM` (MEMTEST) | | `byt=h8 err=h8 adr=h8 dur=h16` |
| `!RD` (DTEST) | | `wr=h12 drp=h8 he=h8 dur=h16` |

### 2.7 Пример (как видит терминал; `recorder.py` переводит в единицы)

```
> PING            < OK PING ver=02 bld=20261201 cal=1 uok=1
> TEL 64          < OK TEL tel=0064                    (100 мс)
> SID 692D3A00    < OK SID sid=692D3A00
> DDRSZ 20        < OK DDRSZ ddr=0020                  (32 МиБ)
> PROF CLR        < OK PROF pn=00
> PROF ADD 2CCCCCCC 00B2D05E00   < OK PROF i=00         (280 МБ/с, 30 с)
> PROF ADD 0 0002FAF080          < OK PROF i=01         (пауза 0,5 с)
> PROF REP 4      < OK PROF pr=0004
> REC PROF        < OK REC sid=692D3A00
                  < !S st=1 md=1 t=0001A2F0  ...  < !S st=2 md=1 t=...
                  < $T v=01 t=... st=2 md=1 cok=1 uok=1 cs=5 ...
> PFAIL           < OK PFAIL
                  < !S st=3 ... < !RP ... tot=...
> READ END        < OK READ ... < !RV ... end=1
```

---

## 3. Формат данных на SSD

### 3.1 Блок 4096 байт = 256 тактов по 16 байт

Little-endian: байт `k` такта = биты `[8k+7:8k]` шины 128 бит. `pos` = (байтовое смещение от START_LBA) / 4096. Слот 128 КиБ = 32 блока.

| Такт | Байты | Поле | Тип | Значение |
|---|---|---|---|---|
| 0 | 0–3 | `magic` | u32 | `0x4352564E` («NVRC») |
| 0 | 4–7 | `sid` | u32 | идентификатор сеанса |
| 0 | 8–15 | `pos` | u64 | номер записанного блока = положение на диске |
| 1 | 16–23 | `seq` | u64 | номер блока источника (с выброшенными); всегда `seq − lost = pos` |
| 1 | 24–31 | `ts` | u64 | такты `ui_clk` от `src_start` до момента, когда стал «должен» такт 0 блока |
| 2 | 32 | `ver` | u8 | 1 |
| 2 | 33 | `flags` | u8 | b0 FIRST, b1 LAST, b2 PAD, b3 TRUNC, b4 PF, b5 DROP (перед блоком выброшены), b6 PROF, b7 BP |
| 2 | 34–35 | `seg` | u16 | сегмент профиля |
| 2 | 36–37 | `rep` | u16 | проход профиля |
| 2 | 38–39 | `valid` | u16 | действительных тактов нагрузки (252; меньше у TRUNC; 0 у PAD) |
| 2 | 40–43 | `lost` | u32 | выброшено блоков в сеансе до этого блока |
| 2 | 44–47 | `ddr_kib` | u32 | заполнение FIFO в КиБ в момент начала блока |
| 3–254 | 48–4079 | нагрузка | 252 такта | такт `j`: `n = pos×256 + j` (32 бит), данные `{~n, n ^ 32'hA5A5A5A5, n×7, n}` |
| 255 | 4080–4083 | `seq_lo` | u32 | `seq[31:0]` |
| 255 | 4084–4087 | `pos_lo` | u32 | `pos[31:0]` |
| 255 | 4088–4091 | `crc` | u32 | CRC-32 zlib (poly 0x04C11DB7 отражённый, init/xorout 0xFFFFFFFF) по байтам 0–4079 |
| 255 | 4092–4095 | `magic_end` | u32 | `0x42444E45` («ENDB») |

* Нагрузка зависит только от адреса (совпадает со старым шаблоном ядра на тех же байтах). Хвост TRUNC и PAD тоже заполняется шаблоном.
* Накладные расходы 1,6 %. `stp`/`RATE` — валовая скорость (байты на диске).
* **PAD и TRUNC — обычные блоки**: каждый увеличивает `seq` и `pos` на 1 (M13).
* Формата RAW в v1 нет (старый шаблон проверяет старый битстрим `ax7a200b_top`).

### 3.2 Конец сеанса

* При `stop`/`pf` текущий блок дописывается немедленно (TRUNC), затем PAD до границы 32 блоков (128 КиБ): ядро не пишет неполный слот. Худший случай +124 КиБ.
* Последний блок сеанса — LAST; после PF у TRUNC/PAD/LAST ещё и PF.
* Нет LAST в конце валидной цепочки = запись оборвана.

### 3.3 Сверка в `rec_verify` (`ui_clk`)

Вход — поток тактов из `vrf_tap` (4.4): `{lba32 слота, смещение в слоте, be, data}`. Позиция такта: `byte = ((lba32 − START_LBA[31:0]) << lbads) + off`, `pos = byte >> 12`, `j = (byte >> 4) & 255` (считается один раз на начало команды, далее инкремент).

| Проверка | Такты | Порядок нужен |
|---|---|---|
| нагрузка = шаблон под `be` | 3–254 | нет |
| `magic`, `sid == rsid`, `pos == pos(адрес)` | 0, только полный такт | нет |
| `ver`, `valid ≤ 252`, `magic_end`, `pos_lo` | 2, 255 | нет |
| `seq − lost == pos`, `seq_lo == seq[31:0]` | 1, 2, 255 | да (контекст слота) |
| CRC-32 | 0–254 | да (состояние CRC слота) |

* Контекст на слот: ожидаемый следующий адрес, `seq`, `lost`, CRC. Такт не по порядку → контекст блока сброшен, проверки «с порядком» пропущены, `ooo++`. Неполный такт (`be≠FFFF`) → `prt++`, проверки полей пропущены, нагрузка проверяется побайтно.
* Итоги: `ok`, `bad`, `crc`, `pe` (такты нагрузки с ошибкой), `fbp` (min позиция плохого такта 0; все F = нет), `mvp` и его `seq/ts/lost`, `pad`, первая ошибка нагрузки (`fep`, `feb`, `exp`, `got` — первый несовпавший DW), `ovf` (переполнение FIFO tap — сверка недействительна).
* `end_found` = есть `fbp`. Дыра = `mvp > fbp` → ошибка.

---

## 4. Архитектура

### 4.1 Структурная схема

```
          link_clk 200 МГц (R4/T4 -> IBUFDS -> BUFG в nvme_host_top = link_clk_out)
 L14 ->[uart_rx]->[rec_cmd_parser]->+----------+--> ctrl_start/stop, verify_mode/cmds, format_req, core_rst_req -> ядро
 L15 <-[uart_tx]<-[rec_tx_fmt]<-----| rec_ctrl |--> таблица профиля, порт A (tdp_ram_2clk 64x72)
 KEY1..4 ->[debounce]-------------->|  (1.3)   |<-- [xadc_mon] (XADC) --temp--> X14 --> MIG device_temp_i
 VIO start/stop/rst --------------->+----------+
                                       ^  |       rec_cdc_bridge (X1..X4, X8, X10, X14)
 ======================================|==|===========================================================
          ui_clk 100 МГц (MIG)          |  v
  [rec_profile]-step->[rec_framer]-AXIS128->[ddr_fifo]-AXIS128-+-> data_ring.s_data (data_clk = ui_clk) -X6-> ядро
       ^ таблица, порт B                     | MIG app 256       +-> [rec_sink_chk] (DTEST)
                                       [mig_ddr3] <-> DDR3 x32 1 ГиБ (банки 34/35)
  [rec_verify] <--X15 async FIFO 512x193-- [vrf_tap]
 =====================================================================================================
          user_clk 125 МГц (pcie_7x): ядро без изменений тракта; nvme_admin + Get Log Page;
          hostmem_writer DATA + slot_lba -> tap-порты ядра -> [vrf_tap] (1 регистр + запись в FIFO)
```

### 4.2 Тактовые домены и сбросы

| Домен | Источник | Частота | Что в нём |
|---|---|---|---|
| `link_clk` | SYS_CLK R4/T4, IBUFDS+BUFG в `nvme_host_top` (`link_clk_out`) | 200 МГц | link_bringup (есть), UART, парсер, форматтер, `rec_ctrl`, `xadc_mon`, кнопки/LED, VIO; он же `sys_clk_i` и опорный 200 МГц MIG |
| `ui_clk` | MIG `ui_clk` (4:1 от 400 МГц) | 100 МГц (`UI_CLK_HZ`) | `rec_profile`, `rec_framer`, `ddr_fifo`, `ddr_memtest`, `rec_sink_chk`, `rec_verify`, вход `data_ring` |
| `user_clk` | pcie_7x | 125 МГц | ядро + Get Log Page + `vrf_tap` (регистр) |

| Сброс | Домен | Источники | Что сбрасывает |
|---|---|---|---|
| `rec_rst` | link | POR (счётчик после конфигурации), кнопка RESET | UART, парсер, форматтер, `rec_ctrl` с конфигурацией и профилем, `xadc_mon` |
| `core_rst_n` → `nvme_host_top.board_rst_n` | link (асинхр. вход ядра) | RESET, VIO `probe_out0`, `rec_ctrl.core_rst_req` | ядро (link_bringup, PCIe, `user_rst`, `data_ring` обе стороны, `link_lost`) |
| `mig_sys_rst_n` | link | только POR | MIG (RESET калибровку не сбрасывает) |
| `ui_rst` | ui | `ui_clk_sync_rst` ИЛИ синхр. `rec_rst` | вся ui-логика; плюс `ui_clear` (импульс) — функциональная очистка |
| `user_rst` | user | как сейчас (IP PCIe) | ядро, `vrf_tap` |

`ui_ok` = `ui_rst` снят (MIG PLL захвачен); `calib` = `init_calib_complete`.

### 4.3 Пересечения доменов

Только через примитивы `rtl/common/` (WP0). Контракт сброса и «возраста» — ниже таблицы.

| # | Из → в | Сигналы | Механизм |
|---|---|---|---|
| X1 | link → ui | `src_stop`, `src_pf`, `ui_clear`, `mt_start`, `dtest_start` | `cdc_pulse` |
| X2 | link → ui | `rec_cfg_t` (bp, sid, rsid, ddr_mib, prof_n/rep/end, mt_mib) — только в IDLE | `cdc_bus` (повтор) |
| X2a | link → ui | `step` (32) | `cdc_bus` разовый; его `dst_upd` = `rate_upd` (M6) |
| X2b | link → ui | `{len_blocks[39:0], prof_en}` | `cdc_bus` разовый; его `dst_upd` = `src_start` (M6) |
| X3 | link → ui | таблица профиля 64×72 | `tdp_ram_2clk`: A пишет link (только IDLE), B читает ui |
| X4 | ui → link | `ui_stat_t` (счётчики источника, DDR, итоги `rec_verify`, `end_found`, `calib`, MEMTEST/DTEST) | `cdc_bus` (повтор) |
| X6 | ui → user | поток данных 128 бит | существующий async FIFO `data_ring` (`data_clk = ui_clk`) |
| X7 | link → user | `ctrl_start/stop`, `verify_mode`, `verify_cmds`, `format_req` | как сейчас: 2 FF в ядре (квазистатика) |
| X8 | link → user | `log_req` | `cdc_pulse` |
| X10 | user → link | `core_stat_t` (состояние, ошибки, счётчики, `beats_in`, `lbads`, `nsze`, `log_smart`, лог-счётчики, `log_busy`, `rtd3e`, `cmd_limit_done`, link) | `cdc_bus` (повтор) |
| X11 | user → link | `status_*` для LED/VIO | как в `ax7a200b_top` |
| X12 | pin → link | `uart_rx` | 2 FF в `uart_rx` |
| X14 | link → ui | `xadc temp[11:0]` → MIG `device_temp_i` | `cdc_bus` (повтор) |
| X15 | user → ui | tap чтения `{lba32, off17, be16, data128}` = 193 бит | `axis_fifo_async` 512 (внутри `vrf_tap`); переполнение → липкий `ovf` → `cdc_sync` |

**Контракт примитивов (I7):**
* `cdc_pulse`: регистр-toggle источника **не** сбрасывается (начальное значение 0 при конфигурации), поэтому сброс источника не рождает импульс. Приёмник в сбросе и в такт выхода из него копирует синхронизированный toggle без импульса. Остановка такта любой стороны — импульсы, выданные во время остановки приёмника, могут быть потеряны (допустимо: все пользователи импульсов повторяют или проверяют состояние).
* `cdc_bus`: без обратного подтверждения. Источник захватывает данные в `hold` и переключает toggle не чаще 1 раза в `HOLD_CYC` своих тактов (`busy` в это время); `HOLD_CYC ≥ ⌈6 × f_src / f_dst⌉ + 2`. Приёмник берёт `hold` через 1 такт после того, как увидел смену синхронизированного toggle, выдаёт `dst_upd` и 8-бит метку `dst_tag`. Сброс любой стороны — как у `cdc_pulse`.
* Возраст: в `link_clk` на каждый принимающий `cdc_bus` (X4, X10) — счётчик тактов с последнего `dst_upd`; > 1 мс (200 000) → `cok`/`uok`=0, данные в решениях `rec_ctrl` не используются (ждать), в `$T` выводятся последние с флагом.

### 4.4 Модули и порты

Типы (`rec_cfg_t`, `ui_stat_t`, `core_stat_t`, `vrf_res_t`, `smart_t`, `xadc_t`, `rec_snap_t`, коды команд/ошибок/состояний, `UI_CLK_HZ`, `PROF_N=64`, функция `crc32_128`) — `rtl/rec/rec_pkg.sv` (WP0). `rec_pkg` не импортирует `nvme_host_pkg`. «→» = выход. Все выходы модулей — регистры.

**WP0 `rtl/common/`**

| Модуль | Параметры | Порты |
|---|---|---|
| `cdc_pulse` | — | `src_clk, src_rst, src_pulse` ; `dst_clk, dst_rst, →dst_pulse` |
| `cdc_bus` | `W, HOLD_CYC, REPEAT` (1 = непрерывный повтор) | `src_clk, src_rst, src_data[W], src_load, →src_busy` ; `dst_clk, dst_rst, →dst_data[W], →dst_upd, →dst_tag[7:0]` |
| `cdc_sync` | `W=1, STAGES=2` | `clk, d[W], →q[W]` (независимые биты / липкие флаги) |
| `tdp_ram_2clk` | `W, DEPTH` | A: `a_clk, a_we, a_addr, a_wdata[W], →a_rdata[W]` ; B: `b_clk, b_en, b_addr, →b_rdata[W]`; латентность 2 |
| `rec_cdc_bridge` (`rtl/rec/`) | — | link: `cfg, step, step_load, start_cfg, start_load, src_stop, src_pf, ui_clear, mt_start, dtest_start, log_req, xadc_temp, →ui_stat, →ui_age_ok, →core_stat, →core_age_ok` ; ui: `ui_clk, ui_rst, →cfg_ui, →step_ui, →rate_upd, →start_cfg_ui, →src_start, →src_stop_ui, →src_pf_ui, →ui_clear_ui, →mt_start_ui, →dtest_start_ui, →device_temp, ui_stat_in` ; user: `user_clk, user_rst, →log_req_u, core_stat_in` |

**WP1 текстовый ввод-вывод (`link_clk`)**

| Модуль | Порты |
|---|---|
| `uart_rx #(CLK_HZ=200e6, BAUD=921600)` | `clk, rst, rxd` (асинхр.), `→m_data[7:0], →m_valid` (1 такт), `→frame_err` (1 такт) |
| `uart_tx #(CLK_HZ, BAUD)` | `clk, rst, s_data[7:0], s_valid, →s_ready, →txd` |
| `rec_cmd_parser` | `clk, rst, rx_data[7:0], rx_valid, rx_ferr` ; `→cmd_valid, cmd_ready, →cmd_op[5:0]` (`rec_op_e`, `OP_PERR`), `→cmd_sub[3:0], →cmd_nargs[1:0], →cmd_arg0[63:0], →cmd_arg1[63:0], →cmd_arg2[63:0], →cmd_err[7:0]` |
| `rec_tx_fmt` | `clk, rst, msg_valid, →msg_ready, msg_kind[4:0], msg_code[7:0], msg_arg0[63:0], msg_arg1[63:0], snap` (`rec_snap_t`, копируется при приёме сообщения), `tel_tick` ; `→txd` через внутренний FIFO 2048 Б и `uart_tx` (экземпляр внутри) |

Форматтер: ПЗУ шаблонов строк = последовательность элементов {литерал | поле (индекс источника в `snap`, ширина в nibble)}; nibble → ASCII. Приоритет: ответ > событие > телеметрия; строка не прерывается. `rec_cmd_parser` разбирает по символу; ключевые слова — ПЗУ ≤ 32 слов.

**WP2 `rec_ctrl` (`link_clk`)**

| Порт | Шир. | Напр. | Смысл |
|---|---|---|---|
| `clk, rst` | 1 | in | link_clk, `rec_rst` |
| `cmd_*` | парсер | in/out | |
| `key_rec, key_stop, key_pf, vio_start, vio_stop` | 1 | in | импульсы после антидребезга |
| `vio_verify` | 1 | in | `vio_start` при 1 = `READ LAST` |
| `msg_valid, msg_ready, msg_kind, msg_code, msg_arg0, msg_arg1` | | out/in | к форматтеру |
| `snap` | `rec_snap_t` | out | |
| `tel_tick` | 1 | out | период `TEL` |
| `ctrl_start, ctrl_stop, verify_mode, format_req` | 1 | out | X7 |
| `verify_cmds` | 32 | out | |
| `core_rst_req` | 1 | out | → `core_rst_n` |
| `log_req` | 1 | out | X8, период `SMART`, только RUN до STOP/PF |
| `cfg` | `rec_cfg_t` | out | X2 |
| `step, step_load` | 32, 1 | out | X2a |
| `start_cfg, start_load` | 41, 1 | out | X2b (`len_blocks` с учётом ёмкости, `prof_en`) |
| `src_stop, src_pf, ui_clear, mt_start, dtest_start` | 1 | out | X1 |
| `ui_stat, ui_age_ok` | `ui_stat_t`, 1 | in | X4 |
| `core_stat, core_age_ok` | `core_stat_t`, 1 | in | X10 |
| `xadc` | `xadc_t` | in | |
| `prof_we, prof_addr[5:0], prof_wdata[71:0], prof_rdata[71:0]` | | out/in | таблица, порт A |
| `led[3:0]` | 4 | out | страница LED (выбор по `key_page`) |
| `key_page` | 1 | in | импульс KEY4 |
| `t_ms` | 32 | out | время от сброса (для `$T`) |

**WP3 источник (`ui_clk`)**

`rec_profile`:

| Порт | Шир. | Напр. | Смысл |
|---|---|---|---|
| `clk, rst, clear` | 1 | in | `clear` = `ui_clear` |
| `start, stop` | 1 | in | `start` = `src_start` |
| `prof_en` | 1 | in | 0 = постоянный `const_step` |
| `nseg` | 7 | in | 1…64 |
| `rep` | 16 | in | 0 = бесконечно |
| `end_stop` | 1 | in | 0 HOLD, 1 STOP |
| `const_step, rate_upd` | 32, 1 | in | X2a |
| `rd_en, rd_addr[5:0]` / `rd_data[71:0]` | | out/in | таблица `{step[31:0], dur[39:0]}`, латентность 2 |
| `step` | 32 | out | |
| `seg_idx` | 6 | out | |
| `rep_cnt` | 16 | out | |
| `prof_active` | 1 | out | |
| `prof_end` | 1 | out | импульс при `end_stop=1` |

Сегмент k держит `step_k` ровно `dur_k` тактов; следующая запись предвыбирается при входе в сегмент; фаза NCO не сбрасывается.

`rec_framer`:

| Порт | Шир. | Напр. | Смысл |
|---|---|---|---|
| `clk, rst, clear` | 1 | in | |
| `start, stop, pf` | 1 | in | |
| `bp_mode, prof_en` | 1 | in | |
| `sid` | 32 | in | |
| `len_blocks` | 40 | in | 0 = без предела (фиксируется по `start`) |
| `step` | 32 | in | |
| `seg_idx[5:0], rep_cnt[15:0]` | | in | |
| `space_ok` | 1 | in | |
| `ddr_fill_kib` | 32 | in | |
| `m_tdata[127:0], m_tvalid, m_tready` | | out/out/in | |
| `src_blocks, wr_blocks` | 48 | out | |
| `drop_blocks` | 32 | out | |
| `pad_blocks, trunc_blocks` | 16 | out | |
| `bp_max_beats` | 40 | out | |
| `rt_stall_max` | 16 | out | RT: максимальный долг внутри принятого блока (норма 0) |
| `len_reached` | 1 | out | импульс |
| `active, done` | 1 | out | `done` = после стопа выдано всё, включая PAD |

Логика: NCO `acc += step`, перенос = такт «должен». RT: на такте 0 нового блока при `space_ok=0` блок выбрасывается целиком (256 «должных» тактов), `seq++`, `lost++`. Внутри принятого блока при `m_tready=0` такты копятся долгом (как BP) — данные не теряются (M11). BP: источник ждёт, долг = `due − sent`. CRC-32 по 128 бит/такт (`crc32_128` из `rec_pkg`). `stop/pf`: TRUNC на полной скорости, затем PAD до кратности 32 блокам, LAST на последнем.

`rec_sink_chk`: `clk, rst, clear, s_tdata[127:0], s_tvalid, →s_tready` (=1), `exp_sid[31:0]` ; `→beats[47:0], →hdr_err[31:0]` (magic/sid/`pos` по порядку).

**WP4 DDR (`ui_clk`)**

`ddr_fifo`:

| Порт | Шир. | Напр. | Смысл |
|---|---|---|---|
| `clk, rst, clear` | 1 | in | `clear` безусловный: прекратить выдачу команд, дождаться всех ответов чтения MIG, обнулить указатели и максимум (≤ 2 мкс) |
| `size_mib` | 11 | in | 1…1024 |
| `s_tdata[127:0], s_tvalid, s_tready` | | in/in/out | вход |
| `m_tdata[127:0], m_tvalid, m_tready` | | out/out/in | выход |
| `space_ok` | 1 | out | свободно в DDR ≥ 8 КиБ **и** свободно в staging ≥ 8 КиБ |
| `fill_bytes, fill_max_bytes` | 31 | out | |
| `idle` | 1 | out | всё пусто, ничего в полёте |
| `wr_bytes, rd_bytes` | 48 | out | |
| `mt_start, mt_mib[10:0]` | | in | |
| `mt_busy, mt_done, mt_err_cnt[31:0], mt_first_err_addr[28:0]` | | out | |
| `app_addr[28:0], app_cmd[2:0], app_en, app_rdy` | | | MIG native |
| `app_wdf_data[255:0], app_wdf_mask[31:0], app_wdf_wren, app_wdf_end, app_wdf_rdy` | | | маска всегда 0 |
| `app_rd_data[255:0], app_rd_data_valid, app_rd_data_end` | | in | |
| `app_sr_req, app_ref_req, app_zq_req` | 1 | out | = 0 |
| `init_calib_complete` | 1 | in | |

Устройство: staging 128×1024 (16 КиБ) → упаковка 2×128→256 → **только целые пачки** по 64 команды BL8 (2 КиБ); выходной FIFO 256×512 (16 КиБ) → распаковка 256→128. Арбитраж (M10): чередовать запись и чтение, когда у обеих сторон есть полная пачка (запись — в staging ≥ 64 слов; чтение — в DDR ≥ 1 пачки и в выходном FIFO места ≥ 64 слов + в полёте); иначе обслуживать ту сторону, что готова. Все объёмы сеанса кратны 2 КиБ (PAD), поэтому хвоста меньше пачки нет. `ddr_memtest` (`clk, rst, start, mib[10:0]`, MIG app через мультиплексор в `ddr_fifo`, `→busy, done, err_cnt[31:0], first_err_addr[28:0]`): два прохода — адрес-как-данные и LFSR-32.

**WP6 сверка**

`vrf_tap` (`user_clk` → `ui_clk`), параметры `N_SLOTS, SLOT_BYTES`:

| Порт | Шир. | Напр. | Смысл |
|---|---|---|---|
| `uclk, urst` | 1 | in | user_clk |
| `wr_en, wr_beat_addr[31:0], wr_data[127:0], wr_be[15:0]` | | in | tap-порты ядра |
| `slot_lba_flat` | N_SLOTS×64 | in | tap-порт ядра |
| `enable` | 1 | in | `verify_mode` (уже синхр. в ядре; tap-порт) |
| `vclk, vrst` | 1 | in | ui_clk |
| `m_tdata[192:0], m_tvalid, m_tready` | | out | `{lba32, off17, be16, data128}` |
| `ovf` | 1 | out (ui) | липкий: FIFO был полон, такт потерян |

user-сторона: один регистр (выбор `slot_lba[slot]` — как ступень S1 `verify_checker`) и запись в `axis_fifo_async #(W=193, DEPTH=512)`. Средний поток ≤ 0,93 ГБ/с против 1,6 ГБ/с выборки в `ui_clk` → FIFO не переполняется.

`rec_verify` (`ui_clk`), параметр `START_LBA`:

| Порт | Шир. | Напр. | Смысл |
|---|---|---|---|
| `clk, rst, clear` | 1 | in | |
| `exp_sid` | 32 | in | |
| `lbads` | 4 | in | из `core_stat` (квазистатика, через `rec_cfg_t`) |
| `s_tdata[192:0], s_tvalid, →s_tready` | | in | от `vrf_tap` |
| `tap_ovf` | 1 | in | |
| `res` | `vrf_res_t` | out | итоги 3.3 |
| `end_found` | 1 | out | |

**WP7 `xadc_mon` (`link_clk`)**: `clk, rst` ; `→xadc` (`xadc_t`: `temp[11:0], vccint[11:0], vccaux[11:0], vccbram[11:0], valid`), `→upd`. Примитив `XADC` напрямую, непрерывная последовательность temp/VCCINT/VCCAUX/VCCBRAM, усреднение 16, DCLK 200 МГц, ADCCLK 25 МГц. `xadc.temp` — тот же 12-бит код, что ждёт MIG `device_temp_i`.

**WP8 `ax7a200b_rec_top`**

| Группа | Порты |
|---|---|
| Тактовые | `sys_clk_p/n` (F10/E10), `free_clk_p/n` (R4/T4) |
| PCIe | `pcie_perst_n` (L16, inout), `pci_exp_txp/txn/rxp/rxn[3:0]` |
| Пользователь | `rst_key_n` (F15), `key_n[3:0]` (L19 L20 K17 J17), `led_n[3:0]` (L13 M13 K14 K13), `uart_tx` (L15), `uart_rx` (L14) |
| Резерв | `ssd_pwr_en` — **J11-3 = G21**, LVCMOS33, константа 1 (будущий ключ питания SSD, активный 1). Тот же вывод — запасной путь PERST# (`AX7A200B_ARCHITECTURE.md` 3.3); он не нужен, т.к. PERST# через L16 работает |
| DDR3 | `ddr3_dq[31:0], ddr3_dqs_p/n[3:0], ddr3_addr[14:0], ddr3_ba[2:0], ddr3_ras_n, ddr3_cas_n, ddr3_we_n, ddr3_reset_n, ddr3_ck_p/n[0], ddr3_cke[0], ddr3_cs_n[0], ddr3_dm[3:0], ddr3_odt[0]` |

Параметры: `SIM_FAST`, `N_SLOTS=4`, `PERST_DRIVE=0`, `ILA_EN=0`, `USE_MIG_MODEL=0` (симуляция: `mig_app_model` вместо `mig_ddr3`).

Кнопки: KEY1 = `REC` (текущая конфигурация), KEY2 = `STOP`, KEY3 = `PFAIL`, KEY4 = страница LED по кругу. LED основной страницы: LED1 link_up, LED2 `calib`, LED3 RUN (мигает) / DONE (горит), LED4 ошибка (горит) или пульс. Отдельный внешний вход PF в v1 не делается (вывод не определён; KEY3 и команда достаточны).

### 4.5 VIO и `hw_status.tcl`

IP `vio_status_a7` не меняется. `probe_in0…18, 22–25` — прежние сигналы ядра; `probe_in19/20/21` = `ddrm/16`, `ddr/16`, `drp`. `probe_out0` → `core_rst_n`; `probe_out1/2` → `rec_ctrl` (`vio_start` = `REC`, или `READ LAST` при `probe_out6=1`; `vio_stop` = `STOP`). Прочие `probe_out` не используются. `hw_status.tcl`: подписи probe 19–21.

### 4.6 Изменения существующих файлов (полный список, всё — WP5, кроме отмеченного)

| Файл | Изменение | Совместимость |
|---|---|---|
| `nvme_host_pkg.sv` | `AOPC_GET_LOG_PAGE = 8'h02`, `LID_SMART = 8'h02`, `SMART_W = 384` | добавление |
| `nvme_admin.sv` | раздел 6.3 | новые входы с умолчанием 0 |
| `nvme_host_core.sv` | проброс портов лога (`log_req` пропускается только при `state ∈ {RUN, DRAIN}`); tap-выходы `tap_wr_en, tap_wr_addr[31:0], tap_wr_data[127:0], tap_wr_be[15:0], tap_slot_lba_flat, tap_verify_mode`; параметр `VERIFY_INTERNAL=1` (0 — `verify_checker` не создаётся, `vfy_*`=0) | умолчания = прежнее поведение |
| `nvme_host_top.sv` | проброс лога, tap, `VERIFY_INTERNAL`; **подключить уже существующие** `stat_beats_in` → новый порт `stat_beats_in[63:0]`, `status_seq_state` → новый порт `stat_seq_state[3:0]` (M17) | `kc705_top`, `ax7a200b_top` без правок |
| `host_sequencer.sv`, `data_ring.sv`, `nvme_write_engine.sv`, `hostmem_*` | **без изменений** | |
| `sim/models/nvme_ssd_model.sv` | Get Log Page 02h (LID 02h, NUMDL, PRP1; содержимое из ручек `smart_*`), ручки `log_status`, `log_delay_cycles`, `stall_at_cmd`/`stall_cycles` | |
| `fpga/scripts/create_project.tcl`, `build.tcl` (WP8) | `board=ax7a200b_rec`: `source fpga/scripts/mig_ip.tcl` (WP4), топ `ax7a200b_rec_top`, XDC `ax7a200b_rec*.xdc` | старые `board=` без изменений |
| `fpga/tools/hw_status.tcl` (WP8) | подписи | |

---

## 5. MIG 7-series и бюджет DDR

### 5.1 Конфигурация IP `mig_ddr3`

| Параметр | Значение |
|---|---|
| Контроллер | 1 × DDR3 SDRAM, native app (AXI выкл.) |
| Устройство | xc7a200t-fbg484-2 |
| Память | Components, `MT41J256M16XX-125` при 1,5 В (U5/U6 = MT41J256M16HA-125, совместима с MT41K; руководство ALINX REV1.0, табл. 2-4-1) |
| Данные | 32 бит (2 × x16), ECC нет, Data Mask есть (не используется) |
| Период памяти | **2500 пс (400 МГц, 800 МТ/с)**; других вариантов в v1 нет |
| PHY : контроллер | 4:1 → `ui_clk` 100 МГц, `app_data` 256, BL8, Sequential |
| Входной период | 5000 пс (200 МГц) |
| System Clock | **No Buffer**, от `link_clk_out` (BUFG) |
| Reference Clock | Use System Clock |
| **XADC Instantiation** | **Disabled**; вход `device_temp_i[11:0]` ← `xadc_mon` через X14 (B1) |
| Address mapping | ROW_BANK_COLUMN |
| Output Driver / RTT_NOM | RZQ/7 / RZQ/4 (60 Ом) |
| Терминация FPGA | UNTUNED_SPLIT_50 (HR, без DCI) |
| Internal Vref | **один параметр в `.prj`**: по демо ALINX; если демо недоступно — включён, 0,750 В |
| sys_rst | активный 0, `mig_sys_rst_n` (только POR) |
| Debug | выкл. |
| Ширина `app_addr` | 29 (rank 1 + bank 3 + row 15 + col 10), шаг 8 на команду BL8 |

### 5.2 Выводы

| Сигнал | Выводы |
|---|---|
| DQS0..3 p/n | E1/D1, K2/J2, M1/L1, P5/P4 |
| DQ0..7 | C2 G1 A1 F3 B2 F1 B1 E2 |
| DQ8..15 | H3 G3 H2 H5 J1 J5 K1 H4 |
| DQ16..23 | L4 M3 L3 J6 K3 K6 J4 L5 |
| DQ24..31 | P1 N4 R1 N2 M6 N5 P6 P2 |
| DM0..3 | D2 G2 M2 M5 |
| A0..14 | AA4 AB2 AA5 AB5 AB1 U3 W1 T1 V2 U2 Y1 W2 Y2 U1 V3 |
| BA0..2 | AA3 Y3 Y4 |
| CS#, RAS#, CAS#, WE# | AB3, V4, W4, AA1 |
| ODT, RESET#, CKE | U5, W6, T5 |
| CK p/n | R3/R2 |

### 5.3 Воспроизводимая генерация (WP4 владеет целиком)

* `fpga/ip/mig_ddr3_a7/mig_ddr3.prj` — XML, пишется автором WP4 по 5.1–5.2 (шаблон — `.prj` примера Xilinx для 7-series DDR3 или демо ALINX, если найдётся). Без GUI.
* `fpga/ip/mig_ddr3_a7/expected_pins.csv` — 5.2 в виде `port,pin`.
* `fpga/scripts/mig_ip.tcl` (вызывается из `create_project.tcl` через `source`, а также отдельно в batch для проверки):
  ```tcl
  create_ip -name mig_7series -vendor xilinx.com -library ip -module_name mig_ddr3 -dir $ip_dir
  file copy -force $repo/fpga/ip/mig_ddr3_a7/mig_ddr3.prj [file join $ip_dir mig_ddr3]
  set_property -dict [list CONFIG.XML_INPUT_FILE {mig_ddr3.prj} CONFIG.RESET_BOARD_INTERFACE {Custom} \
      CONFIG.MIG_DONT_TOUCH_PARAM {Custom} CONFIG.BOARD_MIG_PARAM {Custom}] [get_ips mig_ddr3]
  generate_target all [get_ips mig_ddr3]
  ```
  Проверки (ошибка = `exit 1`): ключевые поля XML `.prj` (`tCK=2500`, `DataWidth=32`, `XADC`=Disabled, `SystemClock=No Buffer`, `ReferenceClock=Use System Clock`, `InternalVref`); `PACKAGE_PIN` из сгенерированного XDC MIG = `expected_pins.csv` (все 71); `generate_target` без CRITICAL WARNING.
* `sim/scripts/run_mig_sanity.tcl` — пример MIG с моделью Micron, вне регрессии.

### 5.4 Бюджет полосы

| Величина | Значение |
|---|---|
| Пик шины 32 бит × 800 МТ/с = порт `app` 256 бит × 100 МГц | 3,20 ГБ/с |
| Пачка 2 КиБ = 256 tCK; смена направления, PRE/ACT, регенерация ≈ 30–40 tCK | эффективность ≈ 80–85 % |
| Доступно (запись + чтение) | ≈ 2,6 ГБ/с |
| Нужно FIFO при R = 847 МБ/с (предел канала) | 2R = 1,7 ГБ/с |
| Предел R по DDR | ≈ 1,3 ГБ/с |
| Пик источника 128 бит × 100 МГц | 1,6 ГБ/с |

Итог: **максимальная устойчивая R без потерь = 847 МБ/с (предел Gen2 x2)**, запас DDR ≈ 1,5×. Измеряется `DTEST` в BP (B4). 32 МиБ при 280 МБ/с перекрывают 0,12 с (пауза ATP 81 мс — 22,7 МБ).

---

## 6. Get Log Page (SMART) параллельно с вводом-выводом

### 6.1 Почему не мешает записи

| Ресурс | Запись данных | Get Log Page |
|---|---|---|
| Очередь | IO SQ1/CQ1, `nvme_write_engine` | Admin SQ0/CQ0, `nvme_admin` |
| Порт запросчика | порт 1 | порт 0 (свободен после ENUM, `req_mux`) |
| Буфер SSD → хост | DATA | IDENT (4 КиБ; Identify уже разобран) |
| Трафик за опрос | — | 2 doorbell + SQE 64 Б + 512 Б + CQE; < 1 КБ/с при 1 с |

`host_sequencer` не меняется: в RUN/DRAIN `nvme_admin` стоит в `S_IDLE`. Одновременная работа портов 0 и 1 `req_mux` и перенос индекса admin-очереди (32 элемента) раньше не исполнялись — покрываются тестами 9.1 (I5).

### 6.2 Команда

SQE: OPC `02h`, NSID `FFFFFFFFh`, PRP1 `IDENT_BASE`, CDW10 = `{NUMDL=16'd127, 4'h0, 4'h0, LID=8'h02}` (RAE=0, LSP=0), CDW11–13 = 0.

### 6.3 Изменения `nvme_admin`

Новые порты:

| Порт | Шир. | Напр. | Смысл |
|---|---|---|---|
| `log_req` | 1 | in | импульс; принимается только при `state==S_IDLE && init_done_r && !in_shutdown && !shutdown_done_r && !shn_pend && !log_dis`; иначе игнорируется |
| `log_busy` | 1 | out | |
| `log_done`, `log_err` | 1 | out | импульсы |
| `log_smart` | 384 | out | раскладка ниже; обновляется только при успехе |
| `log_cnt`, `log_err_cnt` | 16 | out | сброс на `start_init` |
| `log_status` | 16 | out | статус последнего CQE лога |
| `rtd3e_us` | 32 | out | Identify Controller, байты 88–91 |

Раскладка `log_smart` (биты; поле ← байты Log 02h): `[7:0]` CW ← 0; `[23:8]` температура K ← 1–2; `[31:24]` Available Spare ← 3; `[39:32]` Percentage Used ← 5; `[47:40]` 0; `[111:48]` Data Units Read ← 32–39; `[175:112]` Data Units Written ← 48–55; `[207:176]` Power Cycles ← 112–115; `[239:208]` Power On Hours ← 128–131; `[271:240]` Unsafe Shutdowns ← 144–147; `[303:272]` Media Errors ← 160–163; `[335:304]` Warning Temp Time ← 192–195; `[367:336]` Critical Temp Time ← 196–199; `[383:368]` Temp Sensor 1 ← 200–201. Такты `ident_ram`: 0, 2, 3, 7, 8, 9, 10, 12. `rec_pkg::smart_t` (WP0) — packed struct с той же раскладкой.

Правила (I4):
1. Флаг `in_log` (вид команды `K_GET_LOG`, `cmd_kind` 3→4 бита). При `in_log` **все четыре** точки сбоя — ошибка MMIO-ответа (doorbell, `S_MM_RSP`), несовпадение CID, статус CQE ≠ 0, тайм-аут `ADMIN_TIMEOUT_MS` — ведут в `log_err` и `S_IDLE`, не трогая `init_*`, `shutdown_*`, `fail_step`, `cmd_seq`.
2. Успешный CQE лога: `S_AC_CQDB_DONE` → `S_LOG_EX` (чтение 8 тактов, заполнение `log_smart`) → `S_IDLE`; `S_CMD_FINISH` не проходится.
3. После тайм-аута: `log_dis=1` до следующего `start_init`; CID брошенной команды сохраняется в `stale_cid` (`stale_v=1`). Любой следующий опрос CQ (в т.ч. при Delete SQ/CQ выключения), увидевший CQE с `CID == stale_cid`, поглощает его (head++, фаза, CQ doorbell, `stale_v=0`) и продолжает ждать свой CQE. CQE с другим «чужим» CID — по-прежнему `F_CQE_CID`.
4. `start_shutdown` во время лога защёлкивается (`shn_pend`) и исполняется сразу после конца лога.
5. RTD3E: дополнительный `ex_step` в разборе `K_IDENT_CTRL` (такт 5, байты 88–91), до Identify NS (M1).
6. Ядро (`nvme_host_core`) пропускает `log_req` только при `state ∈ {RUN, DRAIN}`; `rec_ctrl` прекращает выдачу после STOP/PF (M2). Если лог всё же шёл в момент PF — флаг `lov` в `!RP`.

---

## 7. Эмуляция пропадания питания

### 7.1 Последовательность

| # | Фаза | Начало → конец | Кто | Оценка |
|---|---|---|---|---|
| 0 | событие | `PFAIL` / KEY3 | `rec_ctrl` | `t0` |
| 1 | стоп источника | `t0` → `src_pf` в `ui_clk` | X1 | ≈ 50 нс |
| 2 | TRUNC + PAD | → `framer.done` | `rec_framer` | ≤ 128 КиБ / 1,6 ГБ/с ≈ 80 мкс |
| 3 | слив DDR | → условие SRC_STOP→CORE_STOP (1.3) | `ddr_fifo`, `data_ring` | `pend / W_ssd` + паузы SSD |
| 4 | слив кольца | `ctrl_stop` → `cs=FLUSH` | ядро DRAIN | ≤ 4 слота + пауза |
| 5 | Flush | FLUSH → SHUTDOWN | ядро | мс |
| 6 | выключение | SHUTDOWN → DONE | ядро | от мс до RTD3E |

Пример: 32 МиБ в DDR, 847 МБ/с → ≈ 40 мс + до 81 мс паузы (ATP). Политика v1 — дописывать всё.

### 7.2 Что измеряется (`!RP`)

Время — счётчик 64 бит `link_clk` (5 нс). `cs` приходит через X10 (обновление ≈ 0,1 мкс) → погрешность фаз ≤ 0,2 мкс.

| Поле | Шир. | Формула |
|---|---|---|
| `src` | h1 | 0 UART, 1 KEY3 |
| `inb`, `lat`, `lov` | h1 | PF в BRINGUP; PF после STOP (`pfLate`); в `t0` шёл Get Log Page |
| `pnd` | h12 | `wr×4096(t0) − ssd(t0)` — принято, но не подтверждено SSD |
| `pdd` | h8 | `ddr(t0)` |
| `prg` | h8 | `(beats_in − beats_base)×16 − ssd`, в `t0` |
| `pad` | h8 | блоки TRUNC+PAD после `t0` |
| `t1`…`t6` | h16 | такты от `t0` до концов фаз 1, 2, 3, 4, 5, 6 (`t6` = всего) |
| `ok` | h1 | DONE, `ce=0`, нет `shutdown_timeout` |
| `rtd` | h8 | RTD3E, мкс |
| `ssd`, `lpos` | h16, h10 | итог на SSD; последняя позиция (для `READ END`) |

Погрешность `pnd` — ±1 команда (128 КиБ).

### 7.3 Что это не моделирует

Питание SSD и FPGA не снимается. `ssd_pwr_en` (G21) зарезервирован под ключ питания SSD. Отчёт даёт требование к удержанию: `E = P_плата × t6`, `C = 2E / (U1² − U2²)`.

---

## 8. Ресурсы, тайминг, ограничения

### 8.1 Бюджет (XC7A200T: 134 600 LUT, 365 RAMB36, 10 MMCM + 10 PLL, 32 BUFG, 285 IOB, 1 XADC)

| Блок | LUT | FF | RAMB36 | Прочее |
|---|---:|---:|---:|---|
| Сейчас (с VIO+ILA) | 10 710 | 15 825 | 179 | MMCM 1, BUFG 8, IOB 13, GTP 4/4 |
| − ILA (`ILA_EN=0`) | −1 800 | −3 500 | −34 | |
| − `verify_checker` (`VERIFY_INTERNAL=0`) | −517 | −1 245 | 0 | освобождает `user_clk` |
| MIG DDR3 x32 | 5 000–7 000 | 6 000–8 000 | 0 | MMCM 1, PLL 1, BUFG 2–3, IDELAYCTRL 2, IOB 71 |
| UART + парсер + форматтер + `rec_ctrl` | 1 500 | 1 500 | 1 (TX FIFO) | |
| `rec_profile` + `rec_framer` + `rec_sink_chk` | 1 500 | 1 800 | 1 (профиль) | |
| `ddr_fifo` + `ddr_memtest` | 1 300 | 1 800 | 8 | |
| `vrf_tap` + `rec_verify` | 2 000 | 3 000 | 3 (FIFO tap) | |
| CDC (≈ 1,5 кбит шин) | 300 | 3 000 | 0 | |
| `xadc_mon` | 150 | 150 | 0 | XADC 1/1 (MIG свой не ставит) |
| Get Log Page в `nvme_admin` | 200 | 450 | 0 | |
| **Итого** | **≈ 21 000–23 000 (16 %)** | **≈ 30 000 (11 %)** | **≈ 158 (43 %)** | MMCM 2, PLL 1, BUFG ≈ 11, IOB 85 |

### 8.2 Риски тайминга

| Риск | Мера |
|---|---|
| `user_clk` WNS +0,062 нс | в `user_clk` только: регистр `vrf_tap` + запись FIFO, логика лога в `nvme_admin`; убираются ILA и `verify_checker`; стратегия `Performance_ExplorePostRoutePhysOpt`; при необходимости pblock |
| MIG и PCIe за область | MIG в банках 34/35, PCIe MMCM у GTP; `report_clock_utilization` |
| `link_clk` 200 МГц | сравнения/сумматоры > 32 бит — с регистром; парсер и форматтер по символу; нет делений и умножений |
| BUFG → PLL MIG | допустимо; проверить DRC и `report_clocks` |
| 800 МТ/с — предел -2 | запасных частот в v1 нет; при сбое калибровки — пересмотр (раздел 12) |

### 8.3 Ограничения

`fpga/constraints/ax7a200b_rec.xdc` (synth+impl): копия `ax7a200b_board.xdc` (в т.ч. `set_false_path -to ASYNC_REG`) + `uart_rx` L14, `ssd_pwr_en` G21 (LVCMOS33). DDR3 — из XDC MIG.

`fpga/constraints/ax7a200b_rec_cdc.xdc` (impl):
```tcl
# link_clk и все такты MIG (производные free_clk) - одна группа; PCIe - другая (M7)
set_clock_groups -asynchronous \
  -group [get_clocks -include_generated_clocks free_clk] \
  -group [get_clocks {txoutclk_x0y0 clk_125mhz_x0y0 clk_250mhz_x0y0 clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0 userclk1 userclk2 mmcm_fb}]
# link <-> ui родственные: пути через примитивы ограничены явно
#   toggle-синхронизаторы имеют ASYNC_REG -> общий false_path; данные cdc_bus (без ASYNC_REG):
set_max_delay -datapath_only 5.0 -from [get_cells -hier -filter {NAME =~ *u_cdc_bus*/hold_q_reg*}] \
                                 -to   [get_cells -hier -filter {NAME =~ *u_cdc_bus*/dst_data_q_reg*}]
# многобитные пересечения: перекос (действует и при false path / clock groups) (I6)
set_bus_skew -from [get_cells -hier -filter {NAME =~ *wr_gray_q_reg*}] -to [get_cells -hier -filter {NAME =~ *wr_gray_sync1_q_reg*}] 4.0
set_bus_skew -from [get_cells -hier -filter {NAME =~ *rd_gray_q_reg*}] -to [get_cells -hier -filter {NAME =~ *rd_gray_sync1_q_reg*}] 4.0
set_bus_skew -from [get_cells -hier -filter {NAME =~ *u_cdc_bus*/hold_q_reg*}] -to [get_cells -hier -filter {NAME =~ *u_cdc_bus*/dst_data_q_reg*}] 4.0
# + блок GTP LOC из ax7a200b_cdc.xdc
```
4,0 нс < минимального периода участвующих доменов (5 нс). Имена регистров `hold_q`, `dst_data_q`, `*_cdc_s0` в примитивах — обязательны (`CODING_RULES.md`). После сборки: `report_cdc` (без непояснённых Critical), `report_bus_skew`, `report_clock_interaction`.

### 8.4 Мощность FPGA (только на ПК, 0 логики FPGA)

| Величина | Статус | Как |
|---|---|---|
| Температура кристалла, VCCINT/AUX/BRAM | **измеряется** | XADC (`xt`, `xvi`, `xva`, `xvb`) |
| Ток / мощность FPGA | **не измеряется** (датчиков нет) | — |
| Мощность FPGA | **оценка** | `fpga/scripts/power_table.tcl` (WP9, офлайн по маршрутизированному DCP): для режимов IDLE, MON, REC 100/280/400/800 МБ/с, READ задаёт `set_switching_activity` (данные ∝ R), для Tj 25/50/70/85 °C пишет `fpga/power/power_table.csv` (`mode,rate_Bps,tj_C,p_static_W,p_dyn_W,p_vccint_W,p_vccaux_W,p_vcco_W,p_mgt_W`) |
| Применение | ПК | `recorder.py` добавляет в CSV `p_fpga_est_W = interp(md, R, xt)` и `p_note=VECTORLESS_ESTIMATE_±30%` |
| Мощность платы | вне FPGA | внешний ваттметр, ручной ввод `recorder.py --board-w` |

---

## 9. План верификации

### 9.1 Модульные тесты (xsim; `sim/tb/tb_<name>.sv` + `.f`; PASS = `TEST PASSED`; у каждого — отрицательная проверка)

| Тест | WP | Проверяет | Отрицательная проверка |
|---|---|---|---|
| `tb_cdc` | 0 | `cdc_pulse/bus/sync`, `tdp_ram_2clk` при 100/125/200 МГц со случайной фазой; ни один импульс не потерян/не удвоен; шина целиком; метка растёт | сброс одной стороны и остановка такта приёмника → нет ложного импульса; `busy` не залипает |
| `tb_uart` | 1 | петля 921600, частота ±2 %, 4 КиБ подряд | испорченный стоп-бит → `frame_err` |
| `tb_cmd_parser` | 1 | все команды 2.3, 1…16 hex-цифр, CR/LF/CRLF, слова-hex (`ADD`) | 17 цифр, 65 символов, лишний аргумент, OVERRUN → точный код |
| `tb_rec_tx_fmt` | 1 | эталонные строки `$T`, `OK`, `ERR`, `!R*` (сравнение побайтно), приоритеты, неразрывность | одновременно 3 источника строк → ни одна не разорвана |
| `tb_rec_ctrl` | 2 | все переходы 1.3 на заглушках, `beats_base`, `log_req` только в RUN до STOP/PF, CORE_RST после ошибки и ABORT, ERROR снимает `ctrl_start` | ядро уходит в FLUSH в SRC_STOP (I3); устаревший снимок (`cok=0`) → решения не принимаются |
| `tb_rec_profile` | 3 | шаг меняется ровно в такт границы; REP 1/3/0; HOLD/STOP; сегмент 1 такт; `rate_upd` | `nseg=0` недопустим (`REC PROF` отсекается выше) — проверка, что движок не стартует |
| `tb_rec_framer` | 3 | поля, CRC = векторы `zlib.crc32` (`sim/vectors/crc_vectors.hex` из `tools`), выброс целым блоком, `seq−lost=pos` (вкл. PAD), TRUNC/PAD до 32, LAST, долг внутри блока при `m_tready=0` | `space_ok` падает посреди блока → блок всё равно цел |
| `tb_ddr_fifo` | 4 | 64 МиБ при случайных `app_rdy/app_wdf_rdy`, заворот, размеры 1/1024 МиБ, `fill/max/space_ok/idle`, только целые пачки, полоса BP ≥ 1,7 ГБ/с в модели без пауз | `clear` при чтениях в полёте → ни одного «старого» такта на выходе |
| `tb_ddr_memtest` | 4 | 0 ошибок | `inject_err_addr` → точный адрес |
| `tb_rec_verify` | 6 | эталонные блоки (`sim/vectors/blocks_*.hex` от WP3): все OK; `vrf_tap` при 125/100,3 МГц с дрожанием | бит в нагрузке; magic/sid/pos; CRC; дыра; `ooo`; неполный такт; переполнение FIFO → `ovf` |
| `tb_xadc_mon` | 7 | UNISIM XADC с `SIM_MONITOR_FILE`: коды, `upd` | вход вне диапазона → коды насыщаются, `valid` |
| `tb_nvme_admin` (доп.) | 5 | Get Log Page: SQE, разбор `log_smart`, RTD3E; **≥ 100 логов подряд (≥ 3 оборота admin-очереди)** при непрерывных doorbell порта 1 | статус ≠ 0; ошибка MMIO-ответа doorbell; CID не тот; тайм-аут + запоздалый CQE во время Delete SQ → поглощён; `start_shutdown` во время лога; `log_req` после SHUTDOWN игнорируется |
| `tb_models` (доп.) | 5 | Log 02h модели | `log_status` ≠ 0 |
| регрессия | 5 | все 22 старых теста PASS | |

### 9.2 Модель DDR

* `sim/models/mig_app_model.sv` (WP4): порты как у `mig_ddr3` (`app_*`, `ui_clk`, `ui_clk_sync_rst`, `init_calib_complete`, `device_temp_i`); параметры `UI_PERIOD_PS` (по умолчанию 10000), `JITTER_PS`; память — ассоциативный массив по 32 Б; ручки `calib_delay`, `calib_fail`, `rd_latency` 20–40, `rdy_drop_pct`, `refresh_period/len`, `bw_limit_pct`, `inject_err_addr`; проверки протокола (данные записи не позже N тактов после команды, `app_en` держится до `app_rdy`, нет команд до калибровки).
* `sim/scripts/run_mig_sanity.tcl` (WP4): пример MIG с моделью Micron, вне регрессии.

### 9.3 Системные сценарии `tb_rec_top` (WP8; `USE_MIG_MODEL=1`, `SIM_FAST=1`, модели PCIe/SSD, UART-BFM)

| # | Сценарий | Критерий |
|---|---|---|
| S1 | `PING`, `CFG?`, `TEL` | строки по эталону |
| S2 | `REC` 4 МиБ, затем `READ LAST` | `lba_mem` = эталон блоков; `!RV ok=… bad=0` |
| S3 | профиль 3 сегмента × REP 2 | `ts` соседних блоков на границе сегмента согласуются с `dur` точно |
| S4 | переполнение: `DDRSZ 1`, `stall_at_cmd` | `drp > 0`; `READ LAST` без ошибок; `lost` последнего блока = `drp` |
| S5 | `PFAIL` в RUN при заполненном DDR | фазы монотонны; `pnd` = `ssd(конец) − ssd(t0) − pad×4096` ± 128 КиБ; `READ END` → `mvp = lpos` |
| S6 | PF в BRINGUP; KEY3; PF после STOP | `inb=1`; `lat=1` |
| S7 | `SMART 1` мс (SIM_FAST) во время записи 64 МиБ | `smo ≥ 100`, `sme=0`; данные как в S2; полоса не ниже S2 более чем на 1 % |
| S8 | `MON`, `FORMAT YES` | SMART без данных; Format выполнен |
| S9 | `DTEST`, `MEMTEST` | `he=0`, `err=0` |
| S10 | `READ END` после Format | `end=1`, `fbp=0` |
| S11 | **I12**: `UI_PERIOD_PS=9970` (100,3 МГц), `JITTER_PS=200`, `REC` 64 МиБ + `READ LAST` | 0 ошибок; `ovf=0` |
| S12 | ядро уходит в FLUSH во время записи (`inject_status` / lba_range) → затем новый `REC` | CORE_RST выполнен; второй сеанс с LBA 0 без «старых» данных (I1) |

### 9.4 Отрицательные проверки (системные)

| Проверка | Ожидание |
|---|---|
| `corrupt_read_at_byte` | `pe=1`, `fep/feb` точно |
| `READ SID` неверный | `fbp=0` |
| порча CRC в `lba_mem` | только `crc=1` |
| `calib_fail` | `REC`/`READ`/`DTEST`/`MEMTEST` → `ERR 05`; `MON`, `FORMAT` работают |
| `log_status` ≠ 0 | `sme` растёт, запись и выключение штатные |
| `ABORT` в RUN | `OK ABORT` получен, CORE_RST, следующий `REC` работает |

### 9.5 ПК

`tools/tests/` (pytest, WP9): разбор строк, перевод единиц (`step`, `dur`, XADC, SMART), CSV, оценка мощности по таблице; `rec_decode.py` на `sim/vectors/blocks_*.hex`; генерация `sim/vectors/crc_vectors.hex`.

---

## 10. Пакеты работ

### 10.1 Таблица

Владение файлами строгое (`CODING_RULES.md`). «Потребляет/выдаёт» — интерфейсы из раздела 4.4.

| WP | Файлы (владеет) | Потребляет | Выдаёт | Тесты | Зависит |
|---|---|---|---|---|---|
| **WP0** CDC и типы | `rtl/rec/rec_pkg.sv`, `rtl/common/cdc_pulse.sv`, `cdc_bus.sv`, `cdc_sync.sv`, `tdp_ram_2clk.sv`, `rtl/rec/rec_cdc_bridge.sv` | — | все типы 4.4, `crc32_128`, примитивы, мост X1–X4/X8/X10/X14 | `tb_cdc` | — (первым) |
| **WP1** текст | `rtl/rec/uart_rx.sv`, `uart_tx.sv`, `rec_cmd_parser.sv`, `rec_tx_fmt.sv` | `rec_pkg` (коды, `rec_snap_t`) | `cmd_*`, `txd`, `msg_ready` | `tb_uart`, `tb_cmd_parser`, `tb_rec_tx_fmt` | WP0 |
| **WP2** контроллер | `rtl/rec/rec_ctrl.sv` | `cmd_*`, `ui_stat`, `core_stat`, `xadc`, кнопки | `msg_*`, `snap`, управление ядром, X1/X2/X2a/X2b/X8, таблица профиля A | `tb_rec_ctrl` | WP0 (порты WP1 зафиксированы здесь) |
| **WP3** источник | `rtl/rec/rec_profile.sv`, `rec_framer.sv`, `rec_sink_chk.sv`, `sim/vectors/blocks_*.hex` | `rec_pkg`, таблица B, `space_ok`, `ddr_fill_kib` | AXIS128, счётчики | `tb_rec_profile`, `tb_rec_framer` | WP0; `crc_vectors.hex` от WP9 (или сам по эталонной функции) |
| **WP4** DDR и MIG | `rtl/rec/ddr_fifo.sv`, `ddr_memtest.sv`, `sim/models/mig_app_model.sv`, `fpga/ip/mig_ddr3_a7/*`, `fpga/scripts/mig_ip.tcl`, `sim/scripts/run_mig_sanity.tcl` | AXIS128, `device_temp_i` | AXIS128, `space_ok`, `fill_*`, MIG app, IP `mig_ddr3` | `tb_ddr_fifo`, `tb_ddr_memtest`, batch `mig_ip.tcl` | WP0 |
| **WP5** ядро | `rtl/nvme_host_pkg.sv`, `rtl/nvme/nvme_admin.sv`, `rtl/nvme/nvme_host_core.sv`, `rtl/nvme_host_top.sv`, `sim/models/nvme_ssd_model.sv`, `sim/tb/tb_nvme_admin.sv`, `sim/tb/tb_models.sv` | — | порты 4.6, 6.3 | `tb_nvme_admin`, `tb_models`, вся регрессия | — (сразу) |
| **WP6** сверка | `rtl/rec/vrf_tap.sv`, `rtl/rec/rec_verify.sv` | tap-порты WP5 (по 4.6, до готовности — по спецификации), `blocks_*.hex` | `vrf_res_t`, `end_found` | `tb_rec_verify` | WP0, WP3 (векторы) |
| **WP7** XADC | `rtl/rec/xadc_mon.sv` | — | `xadc_t` | `tb_xadc_mon` | WP0 |
| **WP8** интеграция | `rtl/board/ax7a200b_rec_top.sv`, `fpga/constraints/ax7a200b_rec.xdc`, `ax7a200b_rec_cdc.xdc`, правки `fpga/scripts/create_project.tcl`, `build.tcl`, `fpga/tools/hw_status.tcl`, `sim/tb/tb_rec_top.*`, `INTEGRATION_NOTES.md` (ведёт) | всё | битстрим | `tb_rec_top` S1–S12 | все |
| **WP9** ПК | `tools/recorder.py`, `tools/rec_decode.py`, `tools/requirements.txt`, `tools/tests/*`, `fpga/scripts/power_table.tcl`, `sim/vectors/crc_vectors.hex` | разделы 2, 3, 8.4 | утилита, векторы CRC, таблица мощности | pytest | — (сразу) |
| **WP10** Opus | ревью стыков, `STAND/`, `docs/`, отчёт | | | | WP8 |

Файлы `.f` и тесты `sim/tb/tb_<x>.*` принадлежат WP своего модуля.

### 10.2 Порядок

1. Сразу: **WP0**, **WP5**, **WP9**.
2. После WP0: **WP1**, **WP2**, **WP3**, **WP4**, **WP7** параллельно; **WP6** — после векторов WP3 (может начать с собственных эталонных блоков по 3.1).
3. **WP8** — после всех; заглушки модулей для раннего `tb_rec_top` допускаются только в его каталоге.
4. Железо (раздел 11) — после S1–S12 PASS и B0.

---

## 11. Ввод в эксплуатацию на стенде

| Этап | Что | Критерий | Если нет |
|---|---|---|---|
| B0 | сборка `board=ax7a200b_rec`; `report_timing_summary`, `report_cdc`, `report_bus_skew` | WNS ≥ 0 во всех доменах | pblock; разбор |
| B1 | UART: `PING`, `recorder.py ping --count 10000` | 0 потерь | 460800 (параметр) |
| B2 | XADC: `STAT?` против Hardware Manager | ≤ 1 код | |
| B3 | MIG: `!B cal=1`, LED2; `MEMTEST` 1 ГиБ × 10; холодный старт × 5 | 0 ошибок | Vref (Q3), затем пересмотр частоты |
| B4 | `DTEST` BP (полоса), RT 800 МБ/с с `DDRSZ 1` | ≥ 1,55 ГБ/с (потолок источника 128 бит × 100 МГц = 1,6 ГБ/с); `he=0`; `drp` растёт при малом DDR | |
| B5 | `MON`, `SMART` | `tk` = `smartctl` ± 1 K | NSID 0 вместо FFFFFFFFh |
| B6 | `REC` 1 ГиБ 200 МБ/с → `READ LAST` | 0 ошибок; чтение ≈ 887 МБ/с | |
| B7 | повтор 10.4/10.9 через DDR: 280 МБ/с весь объём ATP с `DDRSZ 20` (32 МиБ); 800 МБ/с 8 ГиБ | `drp=0`; `ddrm` ≈ прежнему `backlog_max` | |
| B8 | профиль (ступени 50…800 МБ/с, паузы) | `seg/ts` в данных = заданным | |
| B9 | `PFAIL` при 100/400/800 МБ/с и разном заполнении DDR, по 10 раз; `READ END` | `ok=1`; таблица фаз; `mvp = lpos` | |
| B10 | длинная запись, `TEL 3E8`, графики температур и скорости | CSV без пропусков | |

---

## 12. Открытые вопросы (решения по умолчанию действуют до ответа)

| # | Вопрос | По умолчанию |
|---|---|---|
| Q3 | ~~Vref банков 34/35~~ — **закрыт**: вывод VREF банка 34 (V3) занят под DDR3_A[14], значит Internal Vref 0,750 В обязателен (руководство ALINX REV1.0, табл. выводов DDR3). TPS51200 даёт VREF/VTT только микросхемам DDR3 | Internal Vref 0,750 В |
| Q4 | ~~Маркировка~~ — **закрыт**: U5/U6 = MT41J256M16HA-125 (совместима с MT41K256M16HA-125), шина DDR3 1,5 В | MT41J256M16XX-125, 1,5 В |
| Q5 | Вывод резервного ключа питания SSD и внешнего входа PF | `ssd_pwr_en` = J11-3 (G21), константа 1; внешнего входа PF в v1 нет (KEY3 + команда). G21 = J11 вывод 3 подтверждён (руководство ALINX REV1.0, табл. J11; через CON3 вывод B16_L24_P, банк 16, 3,3 В, на J11 последовательно 33 Ом) |
| Q6 | Кнопки | KEY1 REC, KEY2 STOP, KEY3 PF, KEY4 страница LED |
| Q31 | Калибровка MIG на 400 МГц не проходит | пересмотр (запасной частоты в v1 нет) |

Решены в 0.2 и больше не открыты: Q1 (No Buffer от `link_clk_out`), Q2 (только 400 МГц), Q7–Q13, Q15–Q30 — как в таблицах выше; Q14 — дописывать всё; Q17 — сырые коды XADC; Q18 — оценка мощности на ПК; Q21 — профиль 64 сегмента, `END HOLD|STOP`; Q24 — без досрочной записи.

**Примечание (vrf_tap).** FIFO отвода `vrf_tap` на 512 слов безопасен на x2 Gen2, но на x4 может переполниться. Исправление при переходе на x4: гейтировать `hostmem_writer s_mwr_tready` сигналом `tap_afull`.
